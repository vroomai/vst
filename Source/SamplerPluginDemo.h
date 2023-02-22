/*
  ==============================================================================

   This file is part of the JUCE examples.
   Copyright (c) 2022 - Raw Material Software Limited

   The code included in this file is provided under the terms of the ISC license
   http://www.isc.org/downloads/software-support-policy/isc-license. Permission
   To use, copy, modify, and/or distribute this software for any purpose with or
   without fee is hereby granted provided that the above copyright notice and
   this permission notice appear in all copies.

   THE SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY, AND ALL WARRANTIES,
   WHETHER EXPRESSED OR IMPLIED, INCLUDING MERCHANTABILITY AND FITNESS FOR
   PURPOSE, ARE DISCLAIMED.

  ==============================================================================
*/

/*******************************************************************************
 The block below describes the properties of this PIP. A PIP is a short snippet
 of code that can be read by the Projucer and used to generate a JUCE project.

 BEGIN_JUCE_PIP_METADATA

 name:             SamplerPlugin
 version:          1.0.0
 vendor:           JUCE
 website:          http://juce.com
 description:      Sampler audio plugin.

 dependencies:     juce_audio_basics, juce_audio_devices, juce_audio_formats,
                   juce_audio_plugin_client, juce_audio_processors,
                   juce_audio_utils, juce_core, juce_data_structures,
                   juce_events, juce_graphics, juce_gui_basics, juce_gui_extra
 exporters:        xcode_mac, vs2022

 moduleFlags:      JUCE_STRICT_REFCOUNTEDPOINTER=1

 type:             AudioProcessor
 mainClass:        SamplerAudioProcessor

 useLocalCopy:     1

 pluginCharacteristics: pluginIsSynth, pluginWantsMidiIn

 END_JUCE_PIP_METADATA

*******************************************************************************/

#pragma once

#include "DemoUtilities.h"
#include "Gradio.h"

juce::AudioFormatManager formatManager;
std::unique_ptr<juce::AudioFormatReaderSource> readerSource;
juce::AudioTransportSource transportSource;
TransportState state;

#include <array>
#include <atomic>
#include <memory>
#include <vector>
#include <tuple>
#include <iomanip>
#include <sstream>
#include <functional>
#include <mutex>

namespace IDs
{

#define DECLARE_ID(name) const juce::Identifier name (#name);

DECLARE_ID (DATA_MODEL)
DECLARE_ID (sampleReader)
DECLARE_ID (loopMode)
DECLARE_ID (loopPointsSeconds)

DECLARE_ID (MPE_SETTINGS)
DECLARE_ID (synthVoices)
DECLARE_ID (voiceStealingEnabled)
DECLARE_ID (legacyModeEnabled)
DECLARE_ID (mpeZoneLayout)
DECLARE_ID (legacyFirstChannel)
DECLARE_ID (legacyLastChannel)
DECLARE_ID (legacyPitchbendRange)

DECLARE_ID (VISIBLE_RANGE)
DECLARE_ID (totalRange)
DECLARE_ID (visibleRange)

#undef DECLARE_ID

} // namespace IDs

enum class LoopMode
{
    none,
};

// We want to send type-erased commands to the audio thread, but we also
// want those commands to contain move-only resources, so that we can
// construct resources on the gui thread, and then transfer ownership
// cheaply to the audio thread. We can't do this with std::function
// because it enforces that functions are copy-constructible.
// Therefore, we use a very simple templated type-eraser here.
template <typename Proc>
struct Command
{
    virtual ~Command() noexcept                    = default;
    virtual void run (Proc& proc) = 0;
};

template <typename Proc, typename Func>
class TemplateCommand  : public Command<Proc>,
                         private Func
{
public:
    template <typename FuncPrime>
    explicit TemplateCommand (FuncPrime&& funcPrime)
        : Func (std::forward<FuncPrime> (funcPrime))
    {}

    void run (Proc& proc) override { (*this) (proc); }
};

template <typename Proc>
class CommandFifo final
{
public:
    explicit CommandFifo (int size)
        : buffer ((size_t) size),
          abstractFifo (size)
    {}

    CommandFifo()
        : CommandFifo (1024)
    {}

    template <typename Item>
    void push (Item&& item) noexcept
    {
        auto command = makeCommand (std::forward<Item> (item));

        abstractFifo.write (1).forEach ([&] (int index)
        {
            buffer[size_t (index)] = std::move (command);
        });
    }

    void call (Proc& proc) noexcept
    {
        abstractFifo.read (abstractFifo.getNumReady()).forEach ([&] (int index)
        {
            buffer[size_t (index)]->run (proc);
        });
    }

private:
    template <typename Func>
    static std::unique_ptr<Command<Proc>> makeCommand (Func&& func)
    {
        using Decayed = std::decay_t<Func>;
        return std::make_unique<TemplateCommand<Proc, Decayed>> (std::forward<Func> (func));
    }

    std::vector<std::unique_ptr<Command<Proc>>> buffer;
    AbstractFifo abstractFifo;
};

//==============================================================================
// Represents the constant parts of an audio sample: its name, sample rate,
// length, and the audio sample data itself.
// Samples might be pretty big, so we'll keep shared_ptrs to them most of the
// time, to reduce duplication and copying.
class Sample final
{
public:
    Sample (AudioFormatReader& source, double maxSampleLengthSecs)
        : sourceSampleRate (source.sampleRate),
          length (jmin (int (source.lengthInSamples),
                        int (maxSampleLengthSecs * sourceSampleRate))),
          data (jmin (2, int (source.numChannels)), length + 4)
    {
        if (length == 0)
            throw std::runtime_error ("Unable to load sample");

        source.read (&data, 0, length + 4, 0, true, true);
    }

    double getSampleRate() const                    { return sourceSampleRate; }
    int getLength() const                           { return length; }
    const AudioBuffer<float>& getBuffer() const     { return data; }

private:
    double sourceSampleRate;
    int length;
    AudioBuffer<float> data;
};

//==============================================================================
// A class which contains all the information related to sample-playback, such
// as sample data, loop points, and loop kind.
// It is expected that multiple sampler voices will maintain pointers to a
// single instance of this class, to avoid redundant duplication of sample
// data in memory.
class MPESamplerSound final
{
public:
    void setSample (std::unique_ptr<Sample> value)
    {
        sample = move (value);
        setLoopPointsInSeconds (loopPoints);
    }

    Sample* getSample() const
    {
        return sample.get();
    }

    void setLoopPointsInSeconds (Range<double> value)
    {
        loopPoints = sample == nullptr ? value
                                       : Range<double> (0, sample->getLength() / sample->getSampleRate())
                                                        .constrainRange (value);
    }

    Range<double> getLoopPointsInSeconds() const
    {
        return loopPoints;
    }

    void setLoopMode (LoopMode type)
    {
        loopMode = type;
    }

    LoopMode getLoopMode() const
    {
        return loopMode;
    }

private:
    std::unique_ptr<Sample> sample;
    Range<double> loopPoints;
    LoopMode loopMode { LoopMode::none };
};

//==============================================================================
class MPESamplerVoice  : public MPESynthesiserVoice
{
public:
    explicit MPESamplerVoice (std::shared_ptr<const MPESamplerSound> sound)
        : samplerSound (std::move (sound))
    {
        jassert (samplerSound != nullptr);
    }

    void noteStarted() override
    {
        jassert (currentlyPlayingNote.isValid());
        jassert (currentlyPlayingNote.keyState == MPENote::keyDown
              || currentlyPlayingNote.keyState == MPENote::keyDownAndSustained);

        level    .setTargetValue (currentlyPlayingNote.noteOnVelocity.asUnsignedFloat());
        frequency.setTargetValue (currentlyPlayingNote.getFrequencyInHertz());

        auto loopPoints = samplerSound->getLoopPointsInSeconds();
        loopBegin.setTargetValue (loopPoints.getStart() * samplerSound->getSample()->getSampleRate());
        loopEnd  .setTargetValue (loopPoints.getEnd()   * samplerSound->getSample()->getSampleRate());

        for (auto smoothed : { &level, &frequency, &loopBegin, &loopEnd })
            smoothed->reset (currentSampleRate, smoothingLengthInSeconds);

        previousPressure = currentlyPlayingNote.pressure.asUnsignedFloat();
        currentSamplePos = 0.0;
        tailOff          = 0.0;
    }

    void noteStopped (bool allowTailOff) override
    {
        jassert (currentlyPlayingNote.keyState == MPENote::off);

        if (allowTailOff && tailOff == 0.0)
            tailOff = 1.0;
        else
            stopNote();
    }

    void notePressureChanged() override
    {
        const auto currentPressure = static_cast<double> (currentlyPlayingNote.pressure.asUnsignedFloat());
        const auto deltaPressure = currentPressure - previousPressure;
        level.setTargetValue (jlimit (0.0, 1.0, level.getCurrentValue() + deltaPressure));
        previousPressure = currentPressure;
    }

    void notePitchbendChanged() override
    {
        frequency.setTargetValue (currentlyPlayingNote.getFrequencyInHertz());
    }

    void noteTimbreChanged()   override {}
    void noteKeyStateChanged() override {}

    void renderNextBlock (AudioBuffer<float>& outputBuffer,
                          int startSample,
                          int numSamples) override
    {
        render (outputBuffer, startSample, numSamples);
    }

    void renderNextBlock (AudioBuffer<double>& outputBuffer,
                          int startSample,
                          int numSamples) override
    {
        render (outputBuffer, startSample, numSamples);
    }

    double getCurrentSamplePosition() const
    {
        return currentSamplePos;
    }

private:
    template <typename Element>
    void render (AudioBuffer<Element>& outputBuffer, int startSample, int numSamples)
    {
        jassert (samplerSound->getSample() != nullptr);

        auto loopPoints = samplerSound->getLoopPointsInSeconds();
        loopBegin.setTargetValue (loopPoints.getStart() * samplerSound->getSample()->getSampleRate());
        loopEnd  .setTargetValue (loopPoints.getEnd()   * samplerSound->getSample()->getSampleRate());

        auto& data = samplerSound->getSample()->getBuffer();

        auto inL = data.getReadPointer (0);
        auto inR = data.getNumChannels() > 1 ? data.getReadPointer (1) : nullptr;

        auto outL = outputBuffer.getWritePointer (0, startSample);

        if (outL == nullptr)
            return;

        auto outR = outputBuffer.getNumChannels() > 1 ? outputBuffer.getWritePointer (1, startSample)
                                                      : nullptr;

        size_t writePos = 0;

        while (--numSamples >= 0 && renderNextSample (inL, inR, outL, outR, writePos))
            writePos += 1;
    }

    template <typename Element>
    bool renderNextSample (const float* inL,
                           const float* inR,
                           Element* outL,
                           Element* outR,
                           size_t writePos)
    {
        auto currentLevel     = level.getNextValue();
        auto currentFrequency = frequency.getNextValue();
        auto currentLoopBegin = loopBegin.getNextValue();
        auto currentLoopEnd   = loopEnd.getNextValue();

        if (isTailingOff())
        {
            currentLevel *= tailOff;
            tailOff *= 0.9999;

            if (tailOff < 0.005)
            {
                stopNote();
                return false;
            }
        }

        auto pos      = (int) currentSamplePos;
        auto nextPos  = pos + 1;
        auto alpha    = (Element) (currentSamplePos - pos);
        auto invAlpha = 1.0f - alpha;

        // just using a very simple linear interpolation here..
        auto l = static_cast<Element> (currentLevel * (inL[pos] * invAlpha + inL[nextPos] * alpha));
        auto r = static_cast<Element> ((inR != nullptr) ? currentLevel * (inR[pos] * invAlpha + inR[nextPos] * alpha)
                                                        : l);

        if (outR != nullptr)
        {
            outL[writePos] += l;
            outR[writePos] += r;
        }
        else
        {
            outL[writePos] += (l + r) * 0.5f;
        }

        std::tie (currentSamplePos, currentDirection) = getNextState (currentFrequency,
                                                                      currentLoopBegin,
                                                                      currentLoopEnd);

        if (currentSamplePos > samplerSound->getSample()->getLength())
        {
            stopNote();
            return false;
        }

        return true;
    }

    double getSampleValue() const;

    bool isTailingOff() const
    {
        return tailOff != 0.0;
    }

    void stopNote()
    {
        clearCurrentNote();
        currentSamplePos = 0.0;
    }

    enum class Direction
    {
        forward,
        backward
    };

    std::tuple<double, Direction> getNextState (double freq,
                                                double begin,
                                                double end) const
    {
        auto nextPitchRatio = freq / centreFrequencyInHz;

        auto nextSamplePos = currentSamplePos;
        auto nextDirection = currentDirection;

        // Move the current sample pos in the correct direction
        switch (currentDirection)
        {
            case Direction::forward:
                nextSamplePos += nextPitchRatio;
                break;

            case Direction::backward:
                nextSamplePos -= nextPitchRatio;
                break;

            default:
                break;
        }

        // Update current sample position, taking loop mode into account
        // If the loop mode was changed while we were travelling backwards, deal
        // with it gracefully.
        if (nextDirection == Direction::backward && nextSamplePos < begin)
        {
            nextSamplePos = begin;
            nextDirection = Direction::forward;

            return std::tuple<double, Direction> (nextSamplePos, nextDirection);
        }

        if (samplerSound->getLoopMode() == LoopMode::none)
            return std::tuple<double, Direction> (nextSamplePos, nextDirection);

        return std::tuple<double, Direction> (nextSamplePos, nextDirection);
    }

    std::shared_ptr<const MPESamplerSound> samplerSound;
    SmoothedValue<double> level { 0 };
    SmoothedValue<double> frequency { 0 };
    SmoothedValue<double> loopBegin;
    SmoothedValue<double> loopEnd;
    double previousPressure { 0 };
    double currentSamplePos { 0 };
    double tailOff { 0 };
    Direction currentDirection { Direction::forward };
    double smoothingLengthInSeconds { 0.01 };
    double centreFrequencyInHz { 440.0 };
};

template <typename Contents>
class ReferenceCountingAdapter  : public ReferenceCountedObject
{
public:
    template <typename... Args>
    explicit ReferenceCountingAdapter (Args&&... args)
        : contents (std::forward<Args> (args)...)
    {}

    const Contents& get() const
    {
        return contents;
    }

    Contents& get()
    {
        return contents;
    }

private:
    Contents contents;
};

template <typename Contents, typename... Args>
std::unique_ptr<ReferenceCountingAdapter<Contents>>
make_reference_counted (Args&&... args)
{
    auto adapter = new ReferenceCountingAdapter<Contents> (std::forward<Args> (args)...);
    return std::unique_ptr<ReferenceCountingAdapter<Contents>> (adapter);
}

//==============================================================================
inline std::unique_ptr<AudioFormatReader> makeAudioFormatReader (AudioFormatManager& manager,
                                                                 const void* sampleData,
                                                                 size_t dataSize)
{
    return std::unique_ptr<AudioFormatReader> (manager.createReaderFor (std::make_unique<MemoryInputStream> (sampleData,
                                                                                                             dataSize,
                                                                                                             false)));
}

inline std::unique_ptr<AudioFormatReader> makeAudioFormatReader (AudioFormatManager& manager,
                                                                 const File& file)
{
    return std::unique_ptr<AudioFormatReader> (manager.createReaderFor (file));
}

//==============================================================================
class AudioFormatReaderFactory
{
public:
    AudioFormatReaderFactory() = default;
    AudioFormatReaderFactory (const AudioFormatReaderFactory&) = default;
    AudioFormatReaderFactory (AudioFormatReaderFactory&&) = default;
    AudioFormatReaderFactory& operator= (const AudioFormatReaderFactory&) = default;
    AudioFormatReaderFactory& operator= (AudioFormatReaderFactory&&) = default;

    virtual ~AudioFormatReaderFactory() noexcept = default;

    virtual std::unique_ptr<AudioFormatReader> make (AudioFormatManager&) const = 0;
    virtual std::unique_ptr<AudioFormatReaderFactory> clone() const = 0;
};

//==============================================================================
class MemoryAudioFormatReaderFactory  : public AudioFormatReaderFactory
{
public:
    MemoryAudioFormatReaderFactory (const void* sampleDataIn, size_t dataSizeIn)
        : sampleData (sampleDataIn),
          dataSize (dataSizeIn)
    {}

    std::unique_ptr<AudioFormatReader> make (AudioFormatManager& manager) const override
    {
        return makeAudioFormatReader (manager, sampleData, dataSize);
    }

    std::unique_ptr<AudioFormatReaderFactory> clone() const override
    {
        return std::unique_ptr<AudioFormatReaderFactory> (new MemoryAudioFormatReaderFactory (*this));
    }

private:
    const void* sampleData;
    size_t dataSize;
};

//==============================================================================
class FileAudioFormatReaderFactory  : public AudioFormatReaderFactory
{
public:
    explicit FileAudioFormatReaderFactory (File fileIn)
        : file (std::move (fileIn))
    {}

    std::unique_ptr<AudioFormatReader> make (AudioFormatManager& manager) const override
    {
        return makeAudioFormatReader (manager, file);
    }

    std::unique_ptr<AudioFormatReaderFactory> clone() const override
    {
        return std::unique_ptr<AudioFormatReaderFactory> (new FileAudioFormatReaderFactory (*this));
    }

private:
    File file;
};

namespace juce
{

template<>
struct VariantConverter<LoopMode>
{
    static LoopMode fromVar (const var& v)
    {
        return static_cast<LoopMode> (int (v));
    }

    static var toVar (LoopMode loopMode)
    {
        return static_cast<int> (loopMode);
    }
};

template <typename Wrapped>
struct GenericVariantConverter
{
    static Wrapped fromVar (const var& v)
    {
        auto cast = dynamic_cast<ReferenceCountingAdapter<Wrapped>*> (v.getObject());
        jassert (cast != nullptr);
        return cast->get();
    }

    static var toVar (Wrapped range)
    {
        return { make_reference_counted<Wrapped> (std::move (range)).release() };
    }
};

template <typename Numeric>
struct VariantConverter<Range<Numeric>>  : GenericVariantConverter<Range<Numeric>> {};

template<>
struct VariantConverter<MPEZoneLayout>  : GenericVariantConverter<MPEZoneLayout> {};

template<>
struct VariantConverter<std::shared_ptr<AudioFormatReaderFactory>>
    : GenericVariantConverter<std::shared_ptr<AudioFormatReaderFactory>>
{};

} // namespace juce

//==============================================================================
class VisibleRangeDataModel  : private ValueTree::Listener
{
public:
    class Listener
    {
    public:
        virtual ~Listener() noexcept = default;
        virtual void totalRangeChanged   (Range<double>) {}
        virtual void visibleRangeChanged (Range<double>) {}
    };

    VisibleRangeDataModel()
        : VisibleRangeDataModel (ValueTree (IDs::VISIBLE_RANGE))
    {}

    explicit VisibleRangeDataModel (const ValueTree& vt)
        : valueTree (vt),
          totalRange   (valueTree, IDs::totalRange,   nullptr),
          visibleRange (valueTree, IDs::visibleRange, nullptr)
    {
        jassert (valueTree.hasType (IDs::VISIBLE_RANGE));
        valueTree.addListener (this);
    }

    VisibleRangeDataModel (const VisibleRangeDataModel& other)
        : VisibleRangeDataModel (other.valueTree)
    {}

    VisibleRangeDataModel& operator= (const VisibleRangeDataModel& other)
    {
        auto copy (other);
        swap (copy);
        return *this;
    }

    Range<double> getTotalRange() const
    {
        return totalRange;
    }

    void setTotalRange (Range<double> value, UndoManager* undoManager)
    {
        totalRange.setValue (value, undoManager);
        setVisibleRange (visibleRange, undoManager);
    }

    Range<double> getVisibleRange() const
    {
        return visibleRange;
    }

    void setVisibleRange (Range<double> value, UndoManager* undoManager)
    {
        visibleRange.setValue (totalRange.get().constrainRange (value), undoManager);
    }

    void addListener (Listener& listener)
    {
        listenerList.add (&listener);
    }

    void removeListener (Listener& listener)
    {
        listenerList.remove (&listener);
    }

    void swap (VisibleRangeDataModel& other) noexcept
    {
        using std::swap;
        swap (other.valueTree, valueTree);
    }

private:
    void valueTreePropertyChanged (ValueTree&, const Identifier& property) override
    {
        if (property == IDs::totalRange)
        {
            totalRange.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.totalRangeChanged (totalRange); });
        }
        else if (property == IDs::visibleRange)
        {
            visibleRange.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.visibleRangeChanged (visibleRange); });
        }
    }

    void valueTreeChildAdded        (ValueTree&, ValueTree&)      override { jassertfalse; }
    void valueTreeChildRemoved      (ValueTree&, ValueTree&, int) override { jassertfalse; }
    void valueTreeChildOrderChanged (ValueTree&, int, int)        override { jassertfalse; }
    void valueTreeParentChanged     (ValueTree&)                  override { jassertfalse; }

    ValueTree valueTree;

    CachedValue<Range<double>> totalRange;
    CachedValue<Range<double>> visibleRange;

    ListenerList<Listener> listenerList;
};

//==============================================================================
class MPESettingsDataModel  : private ValueTree::Listener
{
public:
    class Listener
    {
    public:
        virtual ~Listener() noexcept = default;
        virtual void synthVoicesChanged (int) {}
        virtual void voiceStealingEnabledChanged (bool) {}
        virtual void legacyModeEnabledChanged (bool) {}
        virtual void mpeZoneLayoutChanged (const MPEZoneLayout&) {}
        virtual void legacyFirstChannelChanged (int) {}
        virtual void legacyLastChannelChanged (int) {}
        virtual void legacyPitchbendRangeChanged (int) {}
    };

    MPESettingsDataModel()
        : MPESettingsDataModel (ValueTree (IDs::MPE_SETTINGS))
    {}

    explicit MPESettingsDataModel (const ValueTree& vt)
        : valueTree (vt),
          synthVoices          (valueTree, IDs::synthVoices,          nullptr, 15),
          voiceStealingEnabled (valueTree, IDs::voiceStealingEnabled, nullptr, false),
          legacyModeEnabled    (valueTree, IDs::legacyModeEnabled,    nullptr, true),
          mpeZoneLayout        (valueTree, IDs::mpeZoneLayout,        nullptr, {}),
          legacyFirstChannel   (valueTree, IDs::legacyFirstChannel,   nullptr, 1),
          legacyLastChannel    (valueTree, IDs::legacyLastChannel,    nullptr, 15),
          legacyPitchbendRange (valueTree, IDs::legacyPitchbendRange, nullptr, 48)
    {
        jassert (valueTree.hasType (IDs::MPE_SETTINGS));
        valueTree.addListener (this);
    }

    MPESettingsDataModel (const MPESettingsDataModel& other)
        : MPESettingsDataModel (other.valueTree)
    {}

    MPESettingsDataModel& operator= (const MPESettingsDataModel& other)
    {
        auto copy (other);
        swap (copy);
        return *this;
    }

    int getSynthVoices() const
    {
        return synthVoices;
    }

    void setSynthVoices (int value, UndoManager* undoManager)
    {
        synthVoices.setValue (Range<int> (1, 20).clipValue (value), undoManager);
    }

    bool getVoiceStealingEnabled() const
    {
        return voiceStealingEnabled;
    }

    void setVoiceStealingEnabled (bool value, UndoManager* undoManager)
    {
        voiceStealingEnabled.setValue (value, undoManager);
    }

    bool getLegacyModeEnabled() const
    {
        return legacyModeEnabled;
    }

    void setLegacyModeEnabled (bool value, UndoManager* undoManager)
    {
        legacyModeEnabled.setValue (value, undoManager);
    }

    MPEZoneLayout getMPEZoneLayout() const
    {
        return mpeZoneLayout;
    }

    void setMPEZoneLayout (MPEZoneLayout value, UndoManager* undoManager)
    {
        mpeZoneLayout.setValue (value, undoManager);
    }

    int getLegacyFirstChannel() const
    {
        return legacyFirstChannel;
    }

    void setLegacyFirstChannel (int value, UndoManager* undoManager)
    {
        legacyFirstChannel.setValue (Range<int> (1, legacyLastChannel).clipValue (value), undoManager);
    }

    int getLegacyLastChannel() const
    {
        return legacyLastChannel;
    }

    void setLegacyLastChannel (int value, UndoManager* undoManager)
    {
        legacyLastChannel.setValue (Range<int> (legacyFirstChannel, 15).clipValue (value), undoManager);
    }

    int getLegacyPitchbendRange() const
    {
        return legacyPitchbendRange;
    }

    void setLegacyPitchbendRange (int value, UndoManager* undoManager)
    {
        legacyPitchbendRange.setValue (Range<int> (0, 95).clipValue (value), undoManager);
    }

    void addListener (Listener& listener)
    {
        listenerList.add (&listener);
    }

    void removeListener (Listener& listener)
    {
        listenerList.remove (&listener);
    }

    void swap (MPESettingsDataModel& other) noexcept
    {
        using std::swap;
        swap (other.valueTree, valueTree);
    }

private:
    void valueTreePropertyChanged (ValueTree&, const Identifier& property) override
    {
        if (property == IDs::synthVoices)
        {
            synthVoices.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.synthVoicesChanged (synthVoices); });
        }
        else if (property == IDs::voiceStealingEnabled)
        {
            voiceStealingEnabled.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.voiceStealingEnabledChanged (voiceStealingEnabled); });
        }
        else if (property == IDs::legacyModeEnabled)
        {
            legacyModeEnabled.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.legacyModeEnabledChanged (legacyModeEnabled); });
        }
        else if (property == IDs::mpeZoneLayout)
        {
            mpeZoneLayout.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.mpeZoneLayoutChanged (mpeZoneLayout); });
        }
        else if (property == IDs::legacyFirstChannel)
        {
            legacyFirstChannel.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.legacyFirstChannelChanged (legacyFirstChannel); });
        }
        else if (property == IDs::legacyLastChannel)
        {
            legacyLastChannel.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.legacyLastChannelChanged (legacyLastChannel); });
        }
        else if (property == IDs::legacyPitchbendRange)
        {
            legacyPitchbendRange.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.legacyPitchbendRangeChanged (legacyPitchbendRange); });
        }
    }

    void valueTreeChildAdded        (ValueTree&, ValueTree&)      override { jassertfalse; }
    void valueTreeChildRemoved      (ValueTree&, ValueTree&, int) override { jassertfalse; }
    void valueTreeChildOrderChanged (ValueTree&, int, int)        override { jassertfalse; }
    void valueTreeParentChanged     (ValueTree&)                  override { jassertfalse; }

    ValueTree valueTree;

    CachedValue<int> synthVoices;
    CachedValue<bool> voiceStealingEnabled;
    CachedValue<bool> legacyModeEnabled;
    CachedValue<MPEZoneLayout> mpeZoneLayout;
    CachedValue<int> legacyFirstChannel;
    CachedValue<int> legacyLastChannel;
    CachedValue<int> legacyPitchbendRange;

    ListenerList<Listener> listenerList;
};

//==============================================================================
class DataModel  : private ValueTree::Listener
{
public:
    class Listener
    {
    public:
        virtual ~Listener() noexcept = default;
        virtual void sampleReaderChanged (std::shared_ptr<AudioFormatReaderFactory>) {}
        virtual void loopModeChanged (LoopMode) {}
        virtual void loopPointsSecondsChanged (Range<double>) {}
    };

    explicit DataModel (AudioFormatManager& audioFormatManagerIn)
        : DataModel (audioFormatManagerIn, ValueTree (IDs::DATA_MODEL))
    {}

    DataModel (AudioFormatManager& audioFormatManagerIn, const ValueTree& vt)
        : audioFormatManager (&audioFormatManagerIn),
          valueTree (vt),
          sampleReader      (valueTree, IDs::sampleReader,      nullptr),
          loopMode          (valueTree, IDs::loopMode,          nullptr, LoopMode::none),
          loopPointsSeconds (valueTree, IDs::loopPointsSeconds, nullptr)
    {
        jassert (valueTree.hasType (IDs::DATA_MODEL));
        valueTree.addListener (this);
    }

    DataModel (const DataModel& other)
        : DataModel (*other.audioFormatManager, other.valueTree)
    {}

    DataModel& operator= (const DataModel& other)
    {
        auto copy (other);
        swap (copy);
        return *this;
    }

    std::unique_ptr<AudioFormatReader> getSampleReader() const
    {
        return sampleReader != nullptr ? sampleReader.get()->make (*audioFormatManager) : nullptr;
    }

    void setSampleReader (std::unique_ptr<AudioFormatReaderFactory> readerFactory,
                          UndoManager* undoManager)
    {
        sampleReader.setValue (move (readerFactory), undoManager);
        setLoopPointsSeconds (Range<double> (0, getSampleLengthSeconds()).constrainRange (loopPointsSeconds),
                              undoManager);
    }

    double getSampleLengthSeconds() const
    {
        if (auto r = getSampleReader())
            return (double) r->lengthInSamples / r->sampleRate;

        return 1.0;
    }

    LoopMode getLoopMode() const
    {
        return loopMode;
    }

    void setLoopMode (LoopMode value, UndoManager* undoManager)
    {
        loopMode.setValue (value, undoManager);
    }

    Range<double> getLoopPointsSeconds() const
    {
        return loopPointsSeconds;
    }

    void setLoopPointsSeconds (Range<double> value, UndoManager* undoManager)
    {
        loopPointsSeconds.setValue (Range<double> (0, getSampleLengthSeconds()).constrainRange (value),
                                    undoManager);
    }

    MPESettingsDataModel mpeSettings()
    {
        return MPESettingsDataModel (valueTree.getOrCreateChildWithName (IDs::MPE_SETTINGS, nullptr));
    }

    void addListener (Listener& listener)
    {
        listenerList.add (&listener);
    }

    void removeListener (Listener& listener)
    {
        listenerList.remove (&listener);
    }

    void swap (DataModel& other) noexcept
    {
        using std::swap;
        swap (other.valueTree, valueTree);
    }

    AudioFormatManager& getAudioFormatManager() const
    {
        return *audioFormatManager;
    }

private:
    void valueTreePropertyChanged (ValueTree&, const Identifier& property) override
    {
        if (property == IDs::sampleReader)
        {
            sampleReader.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.sampleReaderChanged (sampleReader); });
        }
        else if (property == IDs::loopMode)
        {
            loopMode.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.loopModeChanged (loopMode); });
        }
        else if (property == IDs::loopPointsSeconds)
        {
            loopPointsSeconds.forceUpdateOfCachedValue();
            listenerList.call ([this] (Listener& l) { l.loopPointsSecondsChanged (loopPointsSeconds); });
        }
    }

    void valueTreeChildAdded        (ValueTree&, ValueTree&)      override {}
    void valueTreeChildRemoved      (ValueTree&, ValueTree&, int) override { jassertfalse; }
    void valueTreeChildOrderChanged (ValueTree&, int, int)        override { jassertfalse; }
    void valueTreeParentChanged     (ValueTree&)                  override { jassertfalse; }

    AudioFormatManager* audioFormatManager;

    ValueTree valueTree;

    CachedValue<std::shared_ptr<AudioFormatReaderFactory>> sampleReader;
    CachedValue<LoopMode> loopMode;
    CachedValue<Range<double>> loopPointsSeconds;

    ListenerList<Listener> listenerList;
};

namespace
{
void initialiseComboBoxWithConsecutiveIntegers (Component& owner,
                                                ComboBox& comboBox,
                                                Label& label,
                                                int firstValue,
                                                int numValues,
                                                int valueToSelect)
{
    for (auto i = 0; i < numValues; ++i)
        comboBox.addItem (String (i + firstValue), i + 1);

    comboBox.setSelectedId (valueToSelect - firstValue + 1);

    label.attachToComponent (&comboBox, true);
    owner.addAndMakeVisible (comboBox);
}

constexpr int controlHeight     = 24;
constexpr int controlSeparation = 6;

} // namespace

//==============================================================================
class MPELegacySettingsComponent final  : public Component,
                                          private MPESettingsDataModel::Listener
{
public:
    explicit MPELegacySettingsComponent (const MPESettingsDataModel& model,
                                         UndoManager& um)
        : dataModel (model),
          undoManager (&um)
    {
        dataModel.addListener (*this);

        initialiseComboBoxWithConsecutiveIntegers (*this, legacyStartChannel, legacyStartChannelLabel, 1, 16, 1);
        initialiseComboBoxWithConsecutiveIntegers (*this, legacyEndChannel, legacyEndChannelLabel, 1, 16, 16);
        initialiseComboBoxWithConsecutiveIntegers (*this, legacyPitchbendRange, legacyPitchbendRangeLabel, 0, 96, 2);

        legacyStartChannel.onChange = [this]
        {
            if (isLegacyModeValid())
            {
                undoManager->beginNewTransaction();
                dataModel.setLegacyFirstChannel (getFirstChannel(), undoManager);
            }
        };

        legacyEndChannel.onChange = [this]
        {
            if (isLegacyModeValid())
            {
                undoManager->beginNewTransaction();
                dataModel.setLegacyLastChannel (getLastChannel(), undoManager);
            }
        };

        legacyPitchbendRange.onChange = [this]
        {
            if (isLegacyModeValid())
            {
                undoManager->beginNewTransaction();
                dataModel.setLegacyPitchbendRange (legacyPitchbendRange.getText().getIntValue(), undoManager);
            }
        };
    }

    int getMinHeight() const
    {
        return (controlHeight * 3) + (controlSeparation * 2);
    }

private:
    void resized() override
    {
        Rectangle<int> r (proportionOfWidth (0.65f), 0, proportionOfWidth (0.25f), getHeight());

        for (auto& comboBox : { &legacyStartChannel, &legacyEndChannel, &legacyPitchbendRange })
        {
            comboBox->setBounds (r.removeFromTop (controlHeight));
            r.removeFromTop (controlSeparation);
        }
    }

    bool isLegacyModeValid() const
    {
        if (! areLegacyModeParametersValid())
        {
            handleInvalidLegacyModeParameters();
            return false;
        }

        return true;
    }

    void legacyFirstChannelChanged (int value) override
    {
        legacyStartChannel.setSelectedId (value, dontSendNotification);
    }

    void legacyLastChannelChanged (int value) override
    {
        legacyEndChannel.setSelectedId (value, dontSendNotification);
    }

    void legacyPitchbendRangeChanged (int value) override
    {
        legacyPitchbendRange.setSelectedId (value + 1, dontSendNotification);
    }

    int getFirstChannel() const
    {
        return legacyStartChannel.getText().getIntValue();
    }

    int getLastChannel() const
    {
        return legacyEndChannel.getText().getIntValue();
    }

    bool areLegacyModeParametersValid() const
    {
        return getFirstChannel() <= getLastChannel();
    }

    void handleInvalidLegacyModeParameters() const
    {
        AlertWindow::showMessageBoxAsync (AlertWindow::WarningIcon,
                                          "Invalid legacy mode channel layout",
                                          "Cannot set legacy mode start/end channel:\n"
                                          "The end channel must not be less than the start channel!",
                                          "Got it");
    }

    MPESettingsDataModel dataModel;

    ComboBox legacyStartChannel, legacyEndChannel, legacyPitchbendRange;

    Label legacyStartChannelLabel   { {}, "First channel" },
          legacyEndChannelLabel     { {}, "Last channel" },
          legacyPitchbendRangeLabel { {}, "Pitchbend range (semitones)" };

    UndoManager* undoManager;
};

//==============================================================================
class MPENewSettingsComponent final  : public Component,
                                       private MPESettingsDataModel::Listener
{
public:
    MPENewSettingsComponent (const MPESettingsDataModel& model,
                             UndoManager& um)
        : dataModel (model),
          undoManager (&um)
    {
        dataModel.addListener (*this);

        addAndMakeVisible (isLowerZoneButton);
        isLowerZoneButton.setToggleState (true, NotificationType::dontSendNotification);

        initialiseComboBoxWithConsecutiveIntegers (*this, memberChannels, memberChannelsLabel, 0, 16, 15);
        initialiseComboBoxWithConsecutiveIntegers (*this, masterPitchbendRange, masterPitchbendRangeLabel, 0, 96, 2);
        initialiseComboBoxWithConsecutiveIntegers (*this, notePitchbendRange, notePitchbendRangeLabel, 0, 96, 48);

        for (auto& button : { &setZoneButton, &clearAllZonesButton })
            addAndMakeVisible (button);

        setZoneButton.onClick = [this]
        {
            auto isLowerZone = isLowerZoneButton.getToggleState();
            auto numMemberChannels = memberChannels.getText().getIntValue();
            auto perNotePb = notePitchbendRange.getText().getIntValue();
            auto masterPb = masterPitchbendRange.getText().getIntValue();

            if (isLowerZone)
                zoneLayout.setLowerZone (numMemberChannels, perNotePb, masterPb);
            else
                zoneLayout.setUpperZone (numMemberChannels, perNotePb, masterPb);

            undoManager->beginNewTransaction();
            dataModel.setMPEZoneLayout (zoneLayout, undoManager);
        };

        clearAllZonesButton.onClick = [this]
        {
            zoneLayout.clearAllZones();
            undoManager->beginNewTransaction();
            dataModel.setMPEZoneLayout (zoneLayout, undoManager);
        };
    }

    int getMinHeight() const
    {
        return (controlHeight * 6) + (controlSeparation * 6);
    }

private:
    void resized() override
    {
        Rectangle<int> r (proportionOfWidth (0.65f), 0, proportionOfWidth (0.25f), getHeight());

        isLowerZoneButton.setBounds (r.removeFromTop (controlHeight));
        r.removeFromTop (controlSeparation);

        for (auto& comboBox : { &memberChannels, &masterPitchbendRange, &notePitchbendRange })
        {
            comboBox->setBounds (r.removeFromTop (controlHeight));
            r.removeFromTop (controlSeparation);
        }

        r.removeFromTop (controlSeparation);

        auto buttonLeft = proportionOfWidth (0.5f);

        setZoneButton.setBounds (r.removeFromTop (controlHeight).withLeft (buttonLeft));
        r.removeFromTop (controlSeparation);
        clearAllZonesButton.setBounds (r.removeFromTop (controlHeight).withLeft (buttonLeft));
    }

    void mpeZoneLayoutChanged (const MPEZoneLayout& value) override
    {
        zoneLayout = value;
    }

    MPESettingsDataModel dataModel;
    MPEZoneLayout zoneLayout;

    ComboBox memberChannels, masterPitchbendRange, notePitchbendRange;

    ToggleButton isLowerZoneButton  { "Lower zone" };

    Label memberChannelsLabel       { {}, "Nr. of member channels" },
          masterPitchbendRangeLabel { {}, "Master pitchbend range (semitones)" },
          notePitchbendRangeLabel   { {}, "Note pitchbend range (semitones)" };

    TextButton setZoneButton       { "Set zone" },
               clearAllZonesButton { "Clear all zones" };

    UndoManager* undoManager;
};

//==============================================================================
class MPESettingsComponent final  : public Component,
                                    private MPESettingsDataModel::Listener
{
public:
    MPESettingsComponent (const MPESettingsDataModel& model,
                          UndoManager& um)
        : dataModel (model),
          legacySettings (dataModel, um),
          newSettings (dataModel, um),
          undoManager (&um)
    {
        dataModel.addListener (*this);

        addAndMakeVisible (newSettings);
        addChildComponent (legacySettings);

        initialiseComboBoxWithConsecutiveIntegers (*this, numberOfVoices, numberOfVoicesLabel, 1, 20, 15);
        numberOfVoices.onChange = [this]
        {
            undoManager->beginNewTransaction();
            dataModel.setSynthVoices (numberOfVoices.getText().getIntValue(), undoManager);
        };

        for (auto& button : { &legacyModeEnabledToggle, &voiceStealingEnabledToggle })
        {
            addAndMakeVisible (button);
        }

        legacyModeEnabledToggle.onClick = [this]
        {
            undoManager->beginNewTransaction();
            dataModel.setLegacyModeEnabled (legacyModeEnabledToggle.getToggleState(), undoManager);
        };

        voiceStealingEnabledToggle.onClick = [this]
        {
            undoManager->beginNewTransaction();
            dataModel.setVoiceStealingEnabled (voiceStealingEnabledToggle.getToggleState(), undoManager);
        };
    }

private:
    void resized() override
    {
        auto topHeight = jmax (legacySettings.getMinHeight(), newSettings.getMinHeight());
        auto r = getLocalBounds();
        r.removeFromTop (15);
        auto top = r.removeFromTop (topHeight);
        legacySettings.setBounds (top);
        newSettings.setBounds (top);

        r.removeFromLeft (proportionOfWidth (0.65f));
        r = r.removeFromLeft (proportionOfWidth (0.25f));

        auto toggleLeft = proportionOfWidth (0.25f);

        legacyModeEnabledToggle.setBounds (r.removeFromTop (controlHeight).withLeft (toggleLeft));
        r.removeFromTop (controlSeparation);
        voiceStealingEnabledToggle.setBounds (r.removeFromTop (controlHeight).withLeft (toggleLeft));
        r.removeFromTop (controlSeparation);
        numberOfVoices.setBounds (r.removeFromTop (controlHeight));
    }

    void legacyModeEnabledChanged (bool value) override
    {
        legacySettings.setVisible (value);
        newSettings.setVisible (! value);
        legacyModeEnabledToggle.setToggleState (value, dontSendNotification);
    }

    void voiceStealingEnabledChanged (bool value) override
    {
        voiceStealingEnabledToggle.setToggleState (value, dontSendNotification);
    }

    void synthVoicesChanged (int value) override
    {
        numberOfVoices.setSelectedId (value, dontSendNotification);
    }

    MPESettingsDataModel dataModel;
    MPELegacySettingsComponent legacySettings;
    MPENewSettingsComponent newSettings;

    ToggleButton legacyModeEnabledToggle    { "Enable Legacy Mode" },
                 voiceStealingEnabledToggle { "Enable synth voice stealing" };

    ComboBox numberOfVoices;
    Label numberOfVoicesLabel { {}, "Number of synth voices" };

    UndoManager* undoManager;
};

//==============================================================================
class PlaybackPositionOverlay  : public Component,
                                 private Timer,
                                 private VisibleRangeDataModel::Listener
{
public:
    using Provider = std::function<std::vector<float>()>;
    PlaybackPositionOverlay (const VisibleRangeDataModel& model,
                             Provider providerIn)
        : visibleRange (model),
          provider (std::move (providerIn))
    {
        visibleRange.addListener (*this);
        startTimer (16);
    }

private:
    void paint (Graphics& g) override
    {
        g.setColour (Colours::red);

        for (auto position : provider())
        {
            g.drawVerticalLine (roundToInt (timeToXPosition (position)), 0.0f, (float) getHeight());
        }
    }

    void timerCallback() override
    {
        repaint();
    }

    void visibleRangeChanged (Range<double>) override
    {
        repaint();
    }

    double timeToXPosition (double time) const
    {
        return (time - visibleRange.getVisibleRange().getStart()) * getWidth()
                     / visibleRange.getVisibleRange().getLength();
    }

    VisibleRangeDataModel visibleRange;
    Provider provider;
};

//==============================================================================
class WaveformView  : public Component,
                      private ChangeListener,
                      private DataModel::Listener,
                      private VisibleRangeDataModel::Listener
{
public:
    WaveformView (const DataModel& model,
                  const VisibleRangeDataModel& vr)
        : dataModel (model),
          visibleRange (vr),
          thumbnailCache (4),
          thumbnail (4, dataModel.getAudioFormatManager(), thumbnailCache)
    {
        dataModel   .addListener (*this);
        visibleRange.addListener (*this);
        thumbnail   .addChangeListener (this);
    }

private:
    void paint (Graphics& g) override
    {
        
        Path p = Path();
        auto bounds = getLocalBounds();
        p.addRoundedRectangle(bounds.getX(),
                              bounds.getY(),
                              bounds.getWidth(),
                              bounds.getHeight(), 10);
        
        g.reduceClipRegion(p);
        
        // Draw the waveforms
        g.fillAll (Colour(0xFF3B59E6)); // Blue
        
        auto numChannels = thumbnail.getNumChannels();

        if (numChannels == 0)
        {
            g.setColour (Colours::white);
            g.drawFittedText ("Erm, error.", getLocalBounds(), Justification::centred, 1);
            return;
        }

        auto channelHeight = bounds.getHeight() / numChannels;

        for (auto i = 0; i != numChannels; ++i)
        {
            drawChannel (g, i, bounds.removeFromTop (channelHeight));
        }
    }

    void changeListenerCallback (ChangeBroadcaster* source) override
    {
        if (source == &thumbnail)
            repaint();
    }

    void sampleReaderChanged (std::shared_ptr<AudioFormatReaderFactory> value) override
    {
        if (value != nullptr)
        {
            if (auto reader = value->make (dataModel.getAudioFormatManager()))
            {
                thumbnail.setReader (reader.release(), currentHashCode);
                currentHashCode += 1;

                return;
            }
        }

        thumbnail.clear();
    }

    void visibleRangeChanged (Range<double>) override
    {
        repaint();
    }

    void drawChannel (Graphics& g, int channel, Rectangle<int> bounds)
    {
        g.setColour(Colour(0xFFE1E3E8)); // light blue
        //g.setGradientFill (ColourGradient (Colours::lightblue,
        //                                   bounds.getTopLeft().toFloat(),
        //                                   Colours::darkgrey,
        //                                   bounds.getBottomLeft().toFloat(),
        //                                   false));
        
        thumbnail.drawChannel (g,
                               bounds,
                               visibleRange.getVisibleRange().getStart(),
                               visibleRange.getVisibleRange().getEnd(),
                               channel,
                               1.0f);
    }

    DataModel dataModel;
    VisibleRangeDataModel visibleRange;
    AudioThumbnailCache thumbnailCache;
    AudioThumbnail thumbnail;
    int64 currentHashCode = 0;
};

//==============================================================================
class WaveformEditor  : public Component,
                        private DataModel::Listener
{
public:
    WaveformEditor (const DataModel& model,
                    PlaybackPositionOverlay::Provider provider,
                    UndoManager& undoManager)
        : dataModel (model),
          waveformView (model, visibleRange),
          playbackOverlay (visibleRange, move (provider))
    {
        dataModel.addListener (*this);

        addAndMakeVisible (waveformView);
        addAndMakeVisible (playbackOverlay);

        waveformView.toBack();
    }

private:
    void resized() override
    {
        auto bounds = getLocalBounds();
        waveformView   .setBounds (bounds);
        playbackOverlay.setBounds (bounds);
    }

    void sampleReaderChanged (std::shared_ptr<AudioFormatReaderFactory>) override
    {
        auto lengthInSeconds = dataModel.getSampleLengthSeconds();
        visibleRange.setTotalRange   (Range<double> (0, lengthInSeconds), nullptr);
        visibleRange.setVisibleRange (Range<double> (0, lengthInSeconds), nullptr);
    }

    DataModel dataModel;
    VisibleRangeDataModel visibleRange;
    WaveformView waveformView;
    PlaybackPositionOverlay playbackOverlay;
};

class OtherLookAndFeel : public juce::LookAndFeel_V4
{
public:
        
    void drawTextEditorOutline (Graphics& g, int w, int h, TextEditor& te) override
    {
        g.setColour (te.findColour (TextEditor::focusedOutlineColourId));
        g.drawRoundedRectangle(0, 0, w, h, 10, 4);
    }
    
    void fillTextEditorBackground (Graphics& g, int w, int h, TextEditor& te) override
    {
    }
    
    void drawButtonBackground (juce::Graphics& g, juce::Button& button, const juce::Colour& backgroundColour,
                               bool, bool isButtonDown) override
    {
        auto buttonArea = button.getLocalBounds();

        // shadow
        g.setColour ( blueColour );
        g.fillRoundedRectangle(buttonArea.getX(), buttonArea.getY(), buttonArea.getWidth(), buttonArea.getHeight(), 10);

    }
    
    void drawComboBox (Graphics& g, int width, int height, bool,
                                       int, int, int, int, ComboBox& box) override
    {
        auto cornerSize = box.findParentComponentOfClass<ChoicePropertyComponent>() != nullptr ? 0.0f : 3.0f;
        Rectangle<int> boxBounds (0, 0, width, height);

        g.setColour (backgroundColour);
        g.fillRoundedRectangle (boxBounds.toFloat(), cornerSize);

        g.setColour (box.findColour (ComboBox::outlineColourId));
        g.drawRoundedRectangle (boxBounds.toFloat().reduced (0.5f, 0.5f), cornerSize, 1.0f);

        Rectangle<int> arrowZone (width - 30, 0, 20, height);
        Path path;
        path.startNewSubPath ((float) arrowZone.getX() + 3.0f, (float) arrowZone.getCentreY() - 2.0f);
        path.lineTo ((float) arrowZone.getCentreX(), (float) arrowZone.getCentreY() + 3.0f);
        path.lineTo ((float) arrowZone.getRight() - 3.0f, (float) arrowZone.getCentreY() - 2.0f);

        g.setColour (box.findColour (ComboBox::arrowColourId).withAlpha ((box.isEnabled() ? 0.9f : 0.2f)));
        g.strokePath (path, PathStrokeType (2.0f));
    }


    void drawButtonText (juce::Graphics& g, juce::TextButton& button, bool, bool isButtonDown) override
    {
        Font font = getNunitoFont();
        g.setFont(font);

        g.setColour ( highlightColour.withMultipliedAlpha (button.isEnabled() ? 1.0f : 0.5f));

        auto yIndent = juce::jmin (4, button.proportionOfHeight (0.3f));
        auto cornerSize = juce::jmin (button.getHeight(), button.getWidth()) / 2;

        auto fontHeight = juce::roundToInt (font.getHeight() * 0.6f);
        auto leftIndent  = juce::jmin (fontHeight, 2 + cornerSize / (button.isConnectedOnLeft()  ? 4 : 2));
        auto rightIndent = juce::jmin (fontHeight, 2 + cornerSize / (button.isConnectedOnRight() ? 4 : 2));
        auto textWidth = button.getWidth() - leftIndent - rightIndent;

        auto edge = 4;
        auto offset = isButtonDown ? edge / 2 : 0;

        if (textWidth > 0)
            g.drawFittedText (button.getButtonText(),
                              leftIndent + offset, yIndent + offset, textWidth, button.getHeight() - yIndent * 2 - edge,
                              juce::Justification::centred, 2);
    }
        
private:
    static const Font& getNunitoFont()
    {
        static Font nunito (Font (Typeface::createSystemTypefaceFor (BinaryData::NunitoSansRegular_ttf,
             BinaryData::NunitoSansRegular_ttfSize)));
        nunito.setHeight(25.f);
        return nunito;
    }
    
    Colour backgroundColour = Colour(0xFF2D2D2D);
    Colour highlightColour  = Colour(0xFFF8F9FF);
    Colour blueColour       = Colour(0xFF3B59E6);

};


//==============================================================================
class MainSamplerView  : public Component,
                         private DataModel::Listener,
                         private ChangeListener
{
public:
    MainSamplerView (const DataModel& model,
                     PlaybackPositionOverlay::Provider provider,
                     UndoManager& um)
        : dataModel (model),
          waveformEditor (dataModel, move (provider), um),
          undoManager (um)
    {
        dataModel.addListener (*this);
                
        addAndMakeVisible (waveformEditor);
        addAndMakeVisible (submitButton);
        
        
        vroomLabel.setFont(getTinosFont());
        vroomLabel.setText("vroom", juce::NotificationType::dontSendNotification);
        vroomLabel.setColour(Label::textColourId, highlightColour);
        
        aiLabel.setFont(getNunitoFont());
        aiLabel.setText("AI", juce::NotificationType::dontSendNotification);
        aiLabel.setColour(Label::textColourId, highlightColour);
        
        addAndMakeVisible (vroomLabel);
        addAndMakeVisible (aiLabel);
                    
        auto optionsIcon = juce::Drawable::createFromImageData(BinaryData::gear_svg, BinaryData::gear_svgSize);
        auto undoIcon = juce::Drawable::createFromImageData(BinaryData::undo_svg, BinaryData::undo_svgSize);
        auto redoIcon = juce::Drawable::createFromImageData(BinaryData::redo_svg, BinaryData::redo_svgSize);

        
        optionsIcon->replaceColour (Colours::black, highlightColour);
        undoIcon->replaceColour (Colours::black, highlightColour);
        redoIcon->replaceColour (Colours::black, highlightColour);
        
        optionsButton.setImages (optionsIcon.get());
        undoButton.setImages (undoIcon.get());
        redoButton.setImages (redoIcon.get());
        
        addAndMakeVisible (optionsButton);
        addAndMakeVisible (undoButton);
        addAndMakeVisible (redoButton);
                
        addAndMakeVisible(promptInput);
        promptInput.setFont(getNunitoFont());
        promptInput.setText("A loud metal drum in an enormous cathedral.", dontSendNotification);
        promptInput.setMultiLine(true);
        promptInput.setIndents(10, 5);
                
        if (downloadsDir.isDirectory() == false){
            downloadsDir.createDirectory();
        }
         
        auto setFileReader = [&, this](File result) {
            undoManager.beginNewTransaction();
            auto readerFactory = new FileAudioFormatReaderFactory (result);
            
            dataModel.setSampleReader (std::unique_ptr<AudioFormatReaderFactory> (readerFactory),
                                       &undoManager);
            
            generating = false;
            submitButton.setButtonText("Create Sound");
        };
        
        
        submitButton.onClick = [=, this]
        {
            if (generating){
                return;
            }
            
            submitButton.setButtonText("Creating...");
            
            auto thread = new GradioDownloadThread(promptInput.getText(), serverURL, downloadsDir, setFileReader);
            
            thread->startThread();
        };
        
        optionsButton.onClick = [this]{
            open_options();
        };
        
        undoButton.onClick = [this] { undoManager.undo(); };
        redoButton.onClick = [this] { undoManager.redo(); };
        
        changeListenerCallback (&undoManager);
        undoManager.addChangeListener (this);
    }
    
    String get_prompt(){
        return promptInput.getText();
    }
    
    void paint (Graphics& g) override
    {
       g.fillAll(backgroundColour);
    }


    ~MainSamplerView() override
    {
        undoManager.removeChangeListener (this);
    }

private:
    
    static const Font& getNunitoFont()
    {
        static Font nunito (Font (Typeface::createSystemTypefaceFor (BinaryData::NunitoSansRegular_ttf,
             BinaryData::NunitoSansRegular_ttfSize)));
        nunito.setHeight(25.f);
        return nunito;
    }
    
    static const Font& getTinosFont()
    {
        static Font tinos (Font (Typeface::createSystemTypefaceFor (BinaryData::TinosBoldItalic_ttf,
             BinaryData::TinosBoldItalic_ttfSize)));
        tinos.setHeight(65.f);
        return tinos;
    }


    void open_options(){
        
        asyncOptionsWindow = std::make_unique<AlertWindow> ("Cloud Options",
                                                            "",
                                                            MessageBoxIconType::NoIcon);
        asyncOptionsWindow->addTextEditor ("serverUrlInput", serverURL, "Server URL:");
        asyncOptionsWindow->addTextEditor ("downloadsDir", downloadsDir.getFullPathName(), "Download Directory:");
        asyncOptionsWindow->addComboBox ("models", { "audioldm-s-full-v2" }, "Model");
        asyncOptionsWindow->addButton ("OK",     1, KeyPress (KeyPress::returnKey, 0, 0));
        asyncOptionsWindow->addButton ("Cancel", 0, KeyPress (KeyPress::escapeKey, 0, 0));
        
        asyncOptionsWindow->setColour(AlertWindow::ColourIds::backgroundColourId, backgroundColour);
        
        auto lookFeel = dynamic_cast<LookAndFeel_V4*> (&getLookAndFeel());

        lookFeel->setColour(TextButton::buttonColourId, backgroundColour);
        
        asyncOptionsWindow->setLookAndFeel(lookFeel);
        
        asyncOptionsWindow->enterModalState (true,
                                         juce::ModalCallbackFunction::create ([this] (int result)
        {
            if (result > 0)
            {
                serverURL = asyncOptionsWindow->getTextEditorContents("serverUrlInput");
                
                // Check valid directory to save samples
                File potentialValidDir = File(asyncOptionsWindow->getTextEditorContents("downloadsDir"));
                if (potentialValidDir.isDirectory())
                    downloadsDir = potentialValidDir;
            }
            
            asyncOptionsWindow.reset();
        }));
    }

    
    void changeListenerCallback (ChangeBroadcaster* source) override
    {
        if (source == &undoManager)
        {
            undoButton.setEnabled (undoManager.canUndo());
            redoButton.setEnabled (undoManager.canRedo());
        }
    }

    void resized() override
    {
        auto bounds = getLocalBounds();

        auto optionsBar = bounds.removeFromTop (100);
        auto textBar = bounds.removeFromTop (70);
        auto submitBar = bounds.removeFromTop (150);
        auto waveformBar = bounds.removeFromBottom(150);
        
        auto padding = 4;
        auto centreX = bounds.getWidth() / 2;
        int submitButtonWidth = 140;
        int submitButtonHeight = 70;
        int reundoButtonWidth = 30;
        
        optionsButton       .setBounds (bounds.getWidth() - 60,
                                        20,
                                        40,
                                        40);
        
        aiLabel             .setBounds (optionsBar.getX()+optionsBar.getWidth() / 2 + 85,
                                                        -10,
                                                        250,
                                                        80);

        vroomLabel          .setBounds (optionsBar.getX()+optionsBar.getWidth() / 2 - 75,
                                        0,
                                        250,
                                        80);
        
        promptInput.setBounds(textBar.getX(), textBar.getY(), bounds.getWidth() / 1.5, textBar.getHeight());
        promptInput.setCentrePosition(textBar.getCentre());
        
        undoButton          .setBounds (centreX - 100 - reundoButtonWidth,
                                        submitBar.getY() + submitBar.getHeight() / 2 - 5,
                                        reundoButtonWidth, reundoButtonWidth);
        submitButton        .setBounds (centreX - submitButtonWidth / 2,
                                        submitBar.getY() + submitBar.getHeight() / 2 - submitButtonHeight / 4  - 5,
                                        submitButtonWidth, submitButtonHeight);
        
        redoButton          .setBounds (centreX + 100,
                                        submitBar.getY() + submitBar.getHeight() / 2  - 5,
                                        reundoButtonWidth, reundoButtonWidth);

        waveformEditor.setBounds (waveformBar.reduced(20));
    }
    
    //OtherLookAndFeel otherLookAndFeel;
    
    DataModel dataModel;
    WaveformEditor waveformEditor;
    
    std::unique_ptr<AlertWindow> asyncOptionsWindow;

    TextButton submitButton { "Create Sound" };
    
    juce::DrawableButton undoButton { "Undo", juce::DrawableButton::ButtonStyle::ImageFitted };
    juce::DrawableButton redoButton { "Redo", juce::DrawableButton::ButtonStyle::ImageFitted };
    juce::DrawableButton optionsButton { "options", juce::DrawableButton::ButtonStyle::ImageFitted };
    
    Colour backgroundColour = Colour(0xFF2D2D2D);
    Colour highlightColour  = Colour(0xFFF8F9FF);
    Colour blueColour       = Colour(0xFF3B59E6);
    
    String serverURL { "http://127.0.0.1:8000/" };
    File downloadsDir { "~/Library/Audio/Sounds/VroomAI/" };
    
    Label vroomLabel = Label("vroom");
    Label aiLabel    = Label("AI");
            
    bool generating = false;
    
    TextEditor promptInput { "prompt!" };
    
    //FileChooser fileChooser { "Select a file to load...", File(),
    //                          dataModel.getAudioFormatManager().getWildcardForAllFormats() };

    UndoManager& undoManager;
};

//==============================================================================
struct ProcessorState
{
    int synthVoices;
    bool legacyModeEnabled;
    Range<int> legacyChannels;
    int legacyPitchbendRange;
    bool voiceStealingEnabled;
    MPEZoneLayout mpeZoneLayout;
    std::unique_ptr<AudioFormatReaderFactory> readerFactory;
    Range<double> loopPointsSeconds;
    double centreFrequencyHz;
    LoopMode loopMode;
};

//==============================================================================
class SamplerAudioProcessor  : public AudioProcessor
{
public:
    SamplerAudioProcessor()
        : AudioProcessor (BusesProperties().withOutput ("Output", AudioChannelSet::stereo(), true))
    {
        readerFactory.reset (new MemoryAudioFormatReaderFactory (BinaryData::drum_mp4, BinaryData::drum_mp4Size));
        
        // Set up initial sample, which we load from a binary resource
        AudioFormatManager manager;
        manager.registerBasicFormats();
        auto reader = readerFactory->make (manager);
        jassert (reader != nullptr); // Failed to load resource!

        auto sound = samplerSound;
        auto sample = std::unique_ptr<Sample> (new Sample (*reader, 10.0));
        auto lengthInSeconds = sample->getLength() / sample->getSampleRate();
        sound->setLoopPointsInSeconds ({lengthInSeconds * 0.1, lengthInSeconds * 0.9 });
        sound->setSample (move (sample));

        // Start with the max number of voices
        for (auto i = 0; i != maxVoices; ++i)
            synthesiser.addVoice (new MPESamplerVoice (sound));
    }

    void prepareToPlay (double sampleRate, int) override
    {
        synthesiser.setCurrentPlaybackSampleRate (sampleRate);
    }

    void releaseResources() override {}

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override
    {
        return layouts.getMainOutputChannelSet() == AudioChannelSet::mono()
            || layouts.getMainOutputChannelSet() == AudioChannelSet::stereo();
    }

    //==============================================================================
    AudioProcessorEditor* createEditor() override
    {
        // This function will be called from the message thread. We lock the command
        // queue to ensure that no messages are processed for the duration of this
        // call.
        SpinLock::ScopedLockType lock (commandQueueMutex);

        ProcessorState state;
        state.synthVoices          = synthesiser.getNumVoices();
        state.legacyModeEnabled    = synthesiser.isLegacyModeEnabled();
        state.legacyChannels       = synthesiser.getLegacyModeChannelRange();
        state.legacyPitchbendRange = synthesiser.getLegacyModePitchbendRange();
        state.voiceStealingEnabled = synthesiser.isVoiceStealingEnabled();
        state.mpeZoneLayout        = synthesiser.getZoneLayout();
        state.readerFactory        = readerFactory == nullptr ? nullptr : readerFactory->clone();

        auto sound = samplerSound;
        state.loopPointsSeconds = sound->getLoopPointsInSeconds();
        state.loopMode          = sound->getLoopMode();

        return new SamplerAudioProcessorEditor (*this, std::move (state));
    }

    bool hasEditor() const override                                       { return true; }

    //==============================================================================
    const String getName() const override                                 { return "Vroom"; }
    bool acceptsMidi() const override                                     { return true; }
    bool producesMidi() const override                                    { return false; }
    bool isMidiEffect() const override                                    { return false; }
    double getTailLengthSeconds() const override                          { return 0.0; }

    //==============================================================================
    int getNumPrograms() override                                         { return 1; }
    int getCurrentProgram() override                                      { return 0; }
    void setCurrentProgram (int) override                                 {}
    const String getProgramName (int) override                            { return "Vroom"; }
    void changeProgramName (int, const String&) override                  {}

    //==============================================================================
    void getStateInformation (MemoryBlock&) override                      {}
    void setStateInformation (const void*, int) override                  {}

    //==============================================================================
    void processBlock (AudioBuffer<float>& buffer, MidiBuffer& midi) override
    {
        process (buffer, midi);
    }

    void processBlock (AudioBuffer<double>& buffer, MidiBuffer& midi) override
    {
        process (buffer, midi);
    }

    // These should be called from the GUI thread, and will block until the
    // command buffer has enough room to accept a command.
    void setSample (std::unique_ptr<AudioFormatReaderFactory> fact, AudioFormatManager& formatManager)
    {
        class SetSampleCommand
        {
        public:
            SetSampleCommand (std::unique_ptr<AudioFormatReaderFactory> r,
                              std::unique_ptr<Sample> sampleIn,
                              std::vector<std::unique_ptr<MPESamplerVoice>> newVoicesIn)
                : readerFactory (std::move (r)),
                  sample (std::move (sampleIn)),
                  newVoices (std::move (newVoicesIn))
            {}

            void operator() (SamplerAudioProcessor& proc)
            {
                proc.readerFactory = move (readerFactory);
                auto sound = proc.samplerSound;
                sound->setSample (std::move (sample));
                auto numberOfVoices = proc.synthesiser.getNumVoices();
                proc.synthesiser.clearVoices();

                for (auto it = begin (newVoices); proc.synthesiser.getNumVoices() < numberOfVoices; ++it)
                {
                    proc.synthesiser.addVoice (it->release());
                }
            }

        private:
            std::unique_ptr<AudioFormatReaderFactory> readerFactory;
            std::unique_ptr<Sample> sample;
            std::vector<std::unique_ptr<MPESamplerVoice>> newVoices;
        };

        // Note that all allocation happens here, on the main message thread. Then,
        // we transfer ownership across to the audio thread.
        auto loadedSamplerSound = samplerSound;
        std::vector<std::unique_ptr<MPESamplerVoice>> newSamplerVoices;
        newSamplerVoices.reserve (maxVoices);

        for (auto i = 0; i != maxVoices; ++i)
            newSamplerVoices.emplace_back (new MPESamplerVoice (loadedSamplerSound));

        if (fact == nullptr)
        {
            commands.push (SetSampleCommand (move (fact),
                                             nullptr,
                                             move (newSamplerVoices)));
        }
        else if (auto reader = fact->make (formatManager))
        {
            commands.push (SetSampleCommand (move (fact),
                                             std::unique_ptr<Sample> (new Sample (*reader, 10.0)),
                                             move (newSamplerVoices)));
        }
    }

    void setLoopMode (LoopMode loopMode)
    {
        commands.push ([loopMode] (SamplerAudioProcessor& proc)
                       {
                           auto loaded = proc.samplerSound;
                           if (loaded != nullptr)
                               loaded->setLoopMode (loopMode);
                       });
    }

    void setLoopPoints (Range<double> loopPoints)
    {
        commands.push ([loopPoints] (SamplerAudioProcessor& proc)
                       {
                           auto loaded = proc.samplerSound;
                           if (loaded != nullptr)
                               loaded->setLoopPointsInSeconds (loopPoints);
                       });
    }

    void setMPEZoneLayout (MPEZoneLayout layout)
    {
        commands.push ([layout] (SamplerAudioProcessor& proc)
                       {
                           // setZoneLayout will lock internally, so we don't care too much about
                           // ensuring that the layout doesn't get copied or destroyed on the
                           // audio thread. If the audio glitches while updating midi settings
                           // it doesn't matter too much.
                           proc.synthesiser.setZoneLayout (layout);
                       });
    }

    void setLegacyModeEnabled (int pitchbendRange, Range<int> channelRange)
    {
        commands.push ([pitchbendRange, channelRange] (SamplerAudioProcessor& proc)
                       {
                           proc.synthesiser.enableLegacyMode (pitchbendRange, channelRange);
                       });
    }

    void setVoiceStealingEnabled (bool voiceStealingEnabled)
    {
        commands.push ([voiceStealingEnabled] (SamplerAudioProcessor& proc)
                       {
                           proc.synthesiser.setVoiceStealingEnabled (voiceStealingEnabled);
                       });
    }

    void setNumberOfVoices (int numberOfVoices)
    {
        // We don't want to call 'new' on the audio thread. Normally, we'd
        // construct things here, on the GUI thread, and then move them into the
        // command lambda. Unfortunately, C++11 doesn't have extended lambda
        // capture, so we use a custom struct instead.

        class SetNumVoicesCommand
        {
        public:
            SetNumVoicesCommand (std::vector<std::unique_ptr<MPESamplerVoice>> newVoicesIn)
                : newVoices (std::move (newVoicesIn))
            {}

            void operator() (SamplerAudioProcessor& proc)
            {
                if ((int) newVoices.size() < proc.synthesiser.getNumVoices())
                    proc.synthesiser.reduceNumVoices (int (newVoices.size()));
                else
                    for (auto it = begin (newVoices); (size_t) proc.synthesiser.getNumVoices() < newVoices.size(); ++it)
                        proc.synthesiser.addVoice (it->release());
            }

        private:
            std::vector<std::unique_ptr<MPESamplerVoice>> newVoices;
        };

        numberOfVoices = std::min ((int) maxVoices, numberOfVoices);
        auto loadedSamplerSound = samplerSound;
        std::vector<std::unique_ptr<MPESamplerVoice>> newSamplerVoices;
        newSamplerVoices.reserve ((size_t) numberOfVoices);

        for (auto i = 0; i != numberOfVoices; ++i)
            newSamplerVoices.emplace_back (new MPESamplerVoice (loadedSamplerSound));

        commands.push (SetNumVoicesCommand (move (newSamplerVoices)));
    }

    // These accessors are just for an 'overview' and won't give the exact
    // state of the audio engine at a particular point in time.
    // If you call getNumVoices(), get the result '10', and then call
    // getPlaybackPosiiton(9), there's a chance the audio engine will have
    // been updated to remove some voices in the meantime, so the returned
    // value won't correspond to an existing voice.
    int getNumVoices() const                    { return synthesiser.getNumVoices(); }
    float getPlaybackPosition (int voice) const { return playbackPositions.at ((size_t) voice); }

private:
    //==============================================================================
    class SamplerAudioProcessorEditor  : public AudioProcessorEditor,
                                         public FileDragAndDropTarget,
                                         private DataModel::Listener,
                                         private MPESettingsDataModel::Listener
    {
    public:
        SamplerAudioProcessorEditor (SamplerAudioProcessor& p, ProcessorState state)
            : AudioProcessorEditor (&p),
              samplerAudioProcessor (p),
              mainSamplerView (dataModel,
                               [&p]
                               {
                                   std::vector<float> ret;
                                   auto voices = p.getNumVoices();
                                   ret.reserve ((size_t) voices);

                                   for (auto i = 0; i != voices; ++i)
                                       ret.emplace_back (p.getPlaybackPosition (i));

                                   return ret;
                               },
                               undoManager)
        {
            dataModel.addListener (*this);
            mpeSettings.addListener (*this);
            
            formatManager.registerBasicFormats();
            
            //addAndMakeVisible (tabbedComponent);
            addAndMakeVisible (mainSamplerView);
            
            setLookAndFeel (&otherLookAndFeel);
            
            //tabbedComponent.addTab ("Sample Editor", bg, &mainSamplerView, false);
            //tabbedComponent.addTab ("MPE Settings", bg, &settingsComponent, false);

            mpeSettings.setSynthVoices          (state.synthVoices,               nullptr);
            mpeSettings.setLegacyModeEnabled    (state.legacyModeEnabled,         nullptr);
            mpeSettings.setLegacyFirstChannel   (state.legacyChannels.getStart(), nullptr);
            mpeSettings.setLegacyLastChannel    (state.legacyChannels.getEnd(),   nullptr);
            mpeSettings.setLegacyPitchbendRange (state.legacyPitchbendRange,      nullptr);
            mpeSettings.setVoiceStealingEnabled (state.voiceStealingEnabled,      nullptr);
            mpeSettings.setMPEZoneLayout        (state.mpeZoneLayout,             nullptr);

            dataModel.setSampleReader (move (state.readerFactory),    nullptr);
            dataModel.setLoopPointsSeconds  (state.loopPointsSeconds, nullptr);
            dataModel.setLoopMode           (state.loopMode,          nullptr);

            // Make sure that before the constructor has finished, you've set the
            // editor's size to whatever you need it to be.
            setResizable (false, false);
            setResizeLimits (640, 480, 2560, 1440);
            setSize (640, 480);
        }

    private:
        OtherLookAndFeel otherLookAndFeel;
        
        void resized() override
        {
            mainSamplerView.setBounds (getLocalBounds());
        }

        bool keyPressed (const KeyPress& key) override
        {
            if (key == KeyPress ('z', ModifierKeys::commandModifier, 0))
            {
                undoManager.undo();
                return true;
            }

            if (key == KeyPress ('z', ModifierKeys::commandModifier | ModifierKeys::shiftModifier, 0))
            {
                undoManager.redo();
                return true;
            }

            return Component::keyPressed (key);
        }

        bool isInterestedInFileDrag (const StringArray& files) override
        {
            WildcardFileFilter filter (formatManager.getWildcardForAllFormats(), {}, "Known Audio Formats");
            return files.size() == 1 && filter.isFileSuitable (files[0]);
        }

        void filesDropped (const StringArray& files, int, int) override
        {
            jassert (files.size() == 1);
            undoManager.beginNewTransaction();
            auto r = new FileAudioFormatReaderFactory (files[0]);
            dataModel.setSampleReader (std::unique_ptr<AudioFormatReaderFactory> (r),
                                       &undoManager);

        }

        void sampleReaderChanged (std::shared_ptr<AudioFormatReaderFactory> value) override
        {
            samplerAudioProcessor.setSample (value == nullptr ? nullptr : value->clone(),
                                             dataModel.getAudioFormatManager());
        }

        void loopPointsSecondsChanged (Range<double> value) override
        {
            samplerAudioProcessor.setLoopPoints (value);
        }

        void synthVoicesChanged (int value) override
        {
            samplerAudioProcessor.setNumberOfVoices (value);
        }

        void voiceStealingEnabledChanged (bool value) override
        {
            samplerAudioProcessor.setVoiceStealingEnabled (value);
        }

        void legacyModeEnabledChanged (bool value) override
        {
            if (value)
                setProcessorLegacyMode();
            else
                setProcessorMPEMode();
        }

        void mpeZoneLayoutChanged (const MPEZoneLayout&) override
        {
            setProcessorMPEMode();
        }

        void legacyFirstChannelChanged (int) override
        {
            setProcessorLegacyMode();
        }

        void legacyLastChannelChanged (int) override
        {
            setProcessorLegacyMode();
        }

        void legacyPitchbendRangeChanged (int) override
        {
            setProcessorLegacyMode();
        }

        void setProcessorLegacyMode()
        {
            samplerAudioProcessor.setLegacyModeEnabled (mpeSettings.getLegacyPitchbendRange(),
                                                        Range<int> (mpeSettings.getLegacyFirstChannel(),
                                                        mpeSettings.getLegacyLastChannel()));
        }

        void setProcessorMPEMode()
        {
            samplerAudioProcessor.setMPEZoneLayout (mpeSettings.getMPEZoneLayout());
        }
        
        SamplerAudioProcessor& samplerAudioProcessor;
        AudioFormatManager formatManager;
        DataModel dataModel { formatManager };
        UndoManager undoManager;
        MPESettingsDataModel mpeSettings { dataModel.mpeSettings() };

        //TabbedComponent tabbedComponent { TabbedButtonBar::Orientation::TabsAtTop };
        // MPESettingsComponent settingsComponent { dataModel.mpeSettings(), undoManager };
        MainSamplerView mainSamplerView;

        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (SamplerAudioProcessorEditor)
    };

    //==============================================================================
    template <typename Element>
    void process (AudioBuffer<Element>& buffer, MidiBuffer& midiMessages)
    {
        // Try to acquire a lock on the command queue.
        // If we were successful, we pop all pending commands off the queue and
        // apply them to the processor.
        // If we weren't able to acquire the lock, it's because someone called
        // createEditor, which requires that the processor data model stays in
        // a valid state for the duration of the call.
        const GenericScopedTryLock<SpinLock> lock (commandQueueMutex);

        if (lock.isLocked())
            commands.call (*this);

        synthesiser.renderNextBlock (buffer, midiMessages, 0, buffer.getNumSamples());

        auto loadedSamplerSound = samplerSound;

        if (loadedSamplerSound->getSample() == nullptr)
            return;

        auto numVoices = synthesiser.getNumVoices();

        // Update the current playback positions
        for (auto i = 0; i < maxVoices; ++i)
        {
            auto* voicePtr = dynamic_cast<MPESamplerVoice*> (synthesiser.getVoice (i));

            if (i < numVoices && voicePtr != nullptr)
                playbackPositions[(size_t) i] = static_cast<float> (voicePtr->getCurrentSamplePosition() / loadedSamplerSound->getSample()->getSampleRate());
            else
                playbackPositions[(size_t) i] = 0.0f;
        }

    }

    CommandFifo<SamplerAudioProcessor> commands;

    MemoryBlock mb;
    std::unique_ptr<AudioFormatReaderFactory> readerFactory;
    std::shared_ptr<MPESamplerSound> samplerSound = std::make_shared<MPESamplerSound>();
    MPESynthesiser synthesiser;

    // This mutex is used to ensure we don't modify the processor state during
    // a call to createEditor, which would cause the UI to become desynched
    // with the real state of the processor.
    SpinLock commandQueueMutex;

    enum { maxVoices = 20 };

    // This is used for visualising the current playback position of each voice.
    std::array<std::atomic<float>, maxVoices> playbackPositions;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (SamplerAudioProcessor)
};
