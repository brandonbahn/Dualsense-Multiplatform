// Copyright (c) 2025 Rafael Valoto. All Rights Reserved.
// Project: GamepadCore
// Description: Cross-platform library for DualSense and generic gamepad input support.
// Targets: Windows, Linux, macOS.
#pragma once

#include <algorithm>
#include <cstring>
#include <vector>

#if GAMEPAD_CORE_HAS_AUDIO
#include "miniaudio.h"
#endif

#include "GImplementations/Utils/GamepadAudio.h"

using namespace FGamepadAudio;

#if !GAMEPAD_CORE_HAS_AUDIO
struct FAudioDeviceContext {};
#else

/**
 * @brief Audio device context using miniaudio for cross-platform audio playback.
 *
 * This replaces the previous WASAPI-specific implementation to support
 * Windows, Linux, and macOS platforms.
 */
struct FAudioDeviceContext
{
	FAudioDeviceContext() = default;

	~FAudioDeviceContext()
	{
		Close();
	}

	static void DataCallback(ma_device* pDevice, void* pOutput, const void* /*pInput*/, ma_uint32 frameCount)
	{
		auto pContext = static_cast<FAudioDeviceContext*>(pDevice->pUserData);
		if (!pContext || !pContext->bInitialized)
		{
			std::memset(pOutput, 0, frameCount * pDevice->playback.channels * sizeof(float));
			return;
		}

		ma_uint32 framesAvailable = ma_pcm_rb_available_read(&pContext->RingBuffer);
		ma_uint32 framesToRead = frameCount;

		if (framesAvailable < framesToRead)
		{
			framesToRead = framesAvailable;
		}

		if (framesToRead > 0)
		{
			void* pReadBuffer;
			ma_uint32 readSize = framesToRead;
			ma_pcm_rb_acquire_read(&pContext->RingBuffer, &readSize, &pReadBuffer);

			std::memcpy(pOutput, pReadBuffer, readSize * pContext->NumChannels * sizeof(float));

			ma_pcm_rb_commit_read(&pContext->RingBuffer, readSize);
		}

		if (framesToRead < frameCount)
		{
			float* pOutputFloat = static_cast<float*>(pOutput);
			ma_uint32 framesMissing = frameCount - framesToRead;

			std::memset(&pOutputFloat[framesToRead * pContext->NumChannels], 0,
			            framesMissing * pContext->NumChannels * sizeof(float));

			// Underrun: ring buffer didn't have enough data, we filled with zeros.
			// Each zero-fill creates an audible click/pop in the playback stream.
			// Track for throttled logging from the host-side (we can't easily UE_LOG from
			// here without dragging in UE headers into a cross-platform context struct).
			pContext->UnderrunSampleCount += static_cast<int64_t>(framesMissing);
			pContext->UnderrunEventCount += 1;
		}
	}

	bool Initialize(int InSampleRate = 48000, int InNumChannels = 4, int InBufferSizeMs = 1000)
	{
		return InitializeWithDeviceId(nullptr, InSampleRate, InNumChannels, InBufferSizeMs);
	}

	bool InitializeWithDeviceId(const ma_device_id* pDeviceId, int InSampleRate = 48000, int InNumChannels = 4, int InBufferSizeMs = 1000)
	{
		if (bInitialized)
		{
			Close();
		}

		if (pDeviceId)
		{
			DeviceId = *pDeviceId;
			bHasDeviceId = true;
		}
		else
		{
			bHasDeviceId = false;
		}

		SampleRate = InSampleRate;
		NumChannels = InNumChannels;
		BufferSizeMs = InBufferSizeMs;
		ma_uint32 bufferSizeInFrames = static_cast<ma_uint32>(
			(static_cast<long long>(SampleRate) * static_cast<long long>(BufferSizeMs)) / 1000LL);
		if (bufferSizeInFrames < 256)
		{
			bufferSizeInFrames = 256;
		}

		if (ma_pcm_rb_init(ma_format_f32, NumChannels, bufferSizeInFrames, nullptr, nullptr, &RingBuffer) != MA_SUCCESS)
		{
			return false;
		}
		bRingBufferInitialized = true;

		// Pre-fill the ring buffer with silence so CoreAudio has a safe reservoir to read
		// from while the audio-thread producer ramps up. Without this, the first ~1 second
		// of playback has thousands of underrun events because CoreAudio reads from an
		// empty ring buffer immediately after ma_device_start.
		//
		// We pre-fill to HALF the ring buffer size (no upper cap). This puts steady-state
		// at the mid-level, giving maximum jitter tolerance in both directions:
		//   - producer hitch → buffer drains toward empty, has ~half the capacity to absorb
		//   - producer burst → buffer fills toward full, has ~half the capacity to absorb
		//
		// IMPORTANT: ma_pcm_rb_acquire_write only returns the first contiguous writable
		// chunk. If the buffer wraps internally, we'd only fill a fraction of the request
		// in one call. We loop until we've placed the full requested amount (or until
		// acquire stops giving us frames, which shouldn't happen on a freshly-init'd
		// empty buffer but is defensive).
		//
		// Trade-off: latency from "first sample written" to "first sample played" ≈
		// BufferSizeMs / 2. At default 1000ms buffer, that's ~500ms.
		{
			const ma_uint32 prefillFramesTarget = bufferSizeInFrames / 2;
			ma_uint32 prefillFramesActual = 0;
			while (prefillFramesActual < prefillFramesTarget)
			{
				void* pPrefillBuffer = nullptr;
				ma_uint32 framesToFillThisChunk = prefillFramesTarget - prefillFramesActual;
				if (ma_pcm_rb_acquire_write(&RingBuffer, &framesToFillThisChunk, &pPrefillBuffer) != MA_SUCCESS
					|| framesToFillThisChunk == 0)
				{
					break;
				}
				std::memset(pPrefillBuffer, 0,
					static_cast<size_t>(framesToFillThisChunk) * static_cast<size_t>(NumChannels) * sizeof(float));
				ma_pcm_rb_commit_write(&RingBuffer, framesToFillThisChunk);
				prefillFramesActual += framesToFillThisChunk;
			}
			PrefillFramesRequested = prefillFramesTarget;
			PrefillFramesActual = prefillFramesActual;
		}

		ma_device_config Config = ma_device_config_init(ma_device_type_playback);
		Config.playback.format = ma_format_f32;
		Config.playback.channels = NumChannels;
		Config.playback.pDeviceID = pDeviceId;
		Config.sampleRate = SampleRate;
		Config.dataCallback = DataCallback;
		Config.pUserData = this;
		// NOTE: leaving share mode, period size, and period count at miniaudio defaults.
		// Tried:
		//   - Config.periodSizeInMilliseconds = 10 + Config.periods = 3 → made underruns
		//     5x worse on Mac (3-6 events/sec vs ~1 total).
		//   - Config.playback.shareMode = ma_share_mode_exclusive → broke playback entirely
		//     on macOS (no audio reaches the controller).
		// CoreAudio's own negotiated shared-mode behavior is the safest baseline.

		if (ma_device_init(nullptr, &Config, &Device) != MA_SUCCESS)
		{
			ma_pcm_rb_uninit(&RingBuffer);
			bRingBufferInitialized = false;
			return false;
		}

		if (ma_device_start(&Device) != MA_SUCCESS)
		{
			ma_device_uninit(&Device);
			ma_pcm_rb_uninit(&RingBuffer);
			bRingBufferInitialized = false;
			return false;
		}

		bInitialized = true;
		return true;
	}

	void Close()
	{
		if (bInitialized)
		{
			ma_device_uninit(&Device);
			bInitialized = false;
		}
		if (bRingBufferInitialized)
		{
			ma_pcm_rb_uninit(&RingBuffer);
			bRingBufferInitialized = false;
		}
	}

	/**
	 * Re-initialize the device with a new ring-buffer size. Used to change USB latency after the
	 * device has already been opened. Returns false if the device was never initialized.
	 */
	bool ReconfigureRingBuffer(int InBufferSizeMs)
	{
		if (!bInitialized && !bRingBufferInitialized)
		{
			return false;
		}
		const ma_device_id* pDeviceId = bHasDeviceId ? &DeviceId : nullptr;
		return InitializeWithDeviceId(pDeviceId, SampleRate, NumChannels, InBufferSizeMs);
	}

	bool IsValid() const
	{
		return bInitialized && bRingBufferInitialized;
	}

	ma_uint32 GetAvailableWriteFrames()
	{
		if (!bRingBufferInitialized)
		{
			return 0;
		}
		return ma_pcm_rb_available_write(&RingBuffer);
	}

	bool WriteHapticData(const std::vector<std::int16_t>& InterleavedData)
	{
		if (!IsValid() || InterleavedData.empty())
		{
			return false;
		}
		ma_uint32 framesInput = static_cast<ma_uint32>(InterleavedData.size() / 2);

		ma_uint32 framesAvailable = ma_pcm_rb_available_write(&RingBuffer);
		ma_uint32 framesToWrite = (framesInput > framesAvailable) ? framesAvailable : framesInput;

		// Track drops: if we can't fit all input frames, the overflow is silently lost.
		// This appears in playback as missing audio data — audible as clicks/pops.
		if (framesToWrite < framesInput)
		{
			DropSampleCount += static_cast<int64_t>(framesInput - framesToWrite);
			DropEventCount += 1;
		}

		if (framesToWrite == 0)
		{
			return true;
		}

		void* pWriteBufferPtr;
		if (ma_pcm_rb_acquire_write(&RingBuffer, &framesToWrite, &pWriteBufferPtr) != MA_SUCCESS)
		{
			return false;
		}

		float* pOutputBuffer = static_cast<float*>(pWriteBufferPtr);
		constexpr float kNormalization = 1.0f / 32768.0f;
		// Channel layout on the DualSense USB Audio Class device (verified empirically on macOS):
		//   channels 1+2 (indices 0+1) = audio path — firmware routes to 3.5mm jack OR internal
		//                                speaker depending on the audio_flags byte (set by
		//                                ApplyAudioControl via HID, separately from this PCM stream)
		//   channels 3+4 (indices 2+3) = trigger haptic actuators (L2 / R2 voice coils)
		// We always write the audio to 1+2 and zero 3+4 so we don't accidentally drive the
		// triggers as a side-effect of audio playback.
		for (ma_uint32 i = 0; i < framesToWrite; i++)
		{
			float LeftFloat = static_cast<float>(InterleavedData[i * 2]) * kNormalization;
			float RightFloat = static_cast<float>(InterleavedData[(i * 2) + 1]) * kNormalization;
			ma_uint32 baseIndex = i * NumChannels;

			if (NumChannels >= 4)
			{
				pOutputBuffer[baseIndex + 0] = LeftFloat;   // ch 1 — audio L
				pOutputBuffer[baseIndex + 1] = RightFloat;  // ch 2 — audio R
				pOutputBuffer[baseIndex + 2] = 0.f;         // ch 3 — trigger haptic L (silent)
				pOutputBuffer[baseIndex + 3] = 0.f;         // ch 4 — trigger haptic R (silent)
			}
			else
			{
				pOutputBuffer[baseIndex + 0] = LeftFloat;
				pOutputBuffer[baseIndex + 1] = RightFloat;
			}
		}

		ma_pcm_rb_commit_write(&RingBuffer, framesToWrite);
		return true;
	}

public:
	ma_device Device;
	ma_pcm_rb RingBuffer;
	ma_device_id DeviceId;
	int SampleRate = 48000;
	int NumChannels = 4;
	int BufferSizeMs = 1000;
	bool bInitialized = false;
	bool bRingBufferInitialized = false;
	bool bHasDeviceId = false;
	/** Total sample frames the DataCallback had to substitute with silence because the ring
	    buffer was empty. Read + reset by the listener's throttled log to detect underruns. */
	int64_t UnderrunSampleCount = 0;
	int64_t UnderrunEventCount = 0;
	/** Total sample frames WriteHapticData had to discard because the ring buffer was already
	    full. Read + reset by the registry's throttled log. Drops appear as audible glitches
	    because real audio data never makes it to playback. */
	int64_t DropSampleCount = 0;
	int64_t DropEventCount = 0;
	/** Diagnostic: how much pre-fill we asked for vs how much actually landed. If actual
	    is less than requested, the buffer's startup reservoir is short and underruns
	    become more likely. Logged once on init. */
	ma_uint32 PrefillFramesRequested = 0;
	ma_uint32 PrefillFramesActual = 0;
};
#endif
