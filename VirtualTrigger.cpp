#include "AUEffectBase.h"
#include <AudioToolbox/AudioUnitUtilities.h>
#include "VirtualTriggerVersion.h"
#include "VirtualTrigger.h"
#include <CoreMIDI/CoreMIDI.h>
#include <math.h>
#include <mach/mach.h>
#include <mach/mach_time.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#pragma mark ____VirtualTriggerKernel

class VirtualTriggerKernel : public AUKernelBase {
    
public:
	
    VirtualTriggerKernel(AUEffectBase *inAudioUnit );
	virtual ~VirtualTriggerKernel();
    
	// processes one channel of non-interleaved samples
	virtual void Process(const Float32 *inSourceP,
                         Float32 *inDestP,
                         UInt32 inFramesToProcess,
                         UInt32 inNumChannels,
                         bool &ioSilence);
    
	// resets the state
	virtual void Reset();

    struct AudioProcessData GetAudioProcessData();
    
private:
    
    struct AudioProcessData mAudioProcessData;
    
    double mWindowEnergy;
	const Float32 *mPreviousBuffer;
    int mPreviousBufferSize;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#pragma mark ____VirtualTrigger

class VirtualTrigger: public AUEffectBase {
    
public:
	
    VirtualTrigger (AudioUnit component);
    
	virtual OSStatus Version() {return kVirtualTriggerVersion;}
    
    virtual void Cleanup();
    
	virtual OSStatus Initialize();
    
	virtual AUKernelBase *NewKernel() {return new VirtualTriggerKernel(this);}
    
    virtual OSStatus Render(AudioUnitRenderActionFlags & ioActionFlags,
                            AudioTimeStamp const& inTimeStamp,
                            UInt32 inFramesToProcess);
    
	// for custom property
	virtual OSStatus GetPropertyInfo(AudioUnitPropertyID	inID,
                                                AudioUnitScope inScope,
                                                AudioUnitElement inElement,
                                                UInt32 & outDataSize,
                                                Boolean	& outWritable);
    
	virtual OSStatus GetProperty(AudioUnitPropertyID inID,
                                            AudioUnitScope inScope,
                                            AudioUnitElement inElement,
                                            void * outData);
    
    
	virtual OSStatus GetParameterInfo(AudioUnitScope inScope,
                                                 AudioUnitParameterID inParameterID,
                                                 AudioUnitParameterInfo	&outParameterInfo);
    
    virtual void SetParameter(AudioUnitParameterID inID, AudioUnitParameterValue inValue);
    
    // handle presets:
    virtual OSStatus GetPresets(CFArrayRef *outData) const;
    virtual OSStatus NewFactoryPresetSet(const AUPreset & inNewFactoryPreset);
	
	// we'll report a 1ms tail.   A reverb effect would have a much more substantial tail on
	// the order of several seconds....
	//
	virtual	bool SupportsTail() {return false;}
    
	// we have no latency
	//
	// A lookahead compressor or FFT-based processor should report the true latency in seconds
    virtual Float64	GetLatency() {return 0.0;}

protected:
    
private:
    
    int mChannels;
    int mSampleRate;
    int mWaitCount;
    bool mWaiting;
    AudioDisplayData mAudioDisplayData;
    
    MIDIClientRef mMidiClient;
    MIDIEndpointRef mMidiSource;
    MIDIPortRef mMidiOutputPort;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	Standard DSP AudioUnit implementation

AUDIOCOMPONENT_ENTRY(AUBaseProcessFactory, VirtualTrigger)

static int kTriggerModeRms = 0;
static int kTriggerModePeak = 1;

enum {
    kTriggerParam_TriggerNote = 0,
    kTriggerParam_WindowSize = 1,
    kTriggerParam_TriggerThreshold = 2,
    kTriggerParam_TriggerResetDelay = 3,
    kTriggerParam_TriggerOctave = 4,
    kTriggerParam_MidiChannel = 5,
    kTriggerParam_TriggerMode = 6,
    kTriggerParam_Waiting = 7,
    kTriggerParam_InternalMidi = 8,
    kTriggerParam_PlayState = 9
};

enum {
    kNote_C = 0,
    kNote_CSharp = 1,
    kNote_D = 2,
    kNote_DSharp = 3,
    kNote_E = 4,
    kNote_F = 5,
    kNote_FSharp = 6,
    kNote_G = 7,
    kNote_GSharp = 8,
    kNote_A = 9,
    kNote_ASharp = 10,
    kNote_B = 11
};

static CFStringRef kWindowSize_Name = CFSTR("Window Size");
static CFStringRef kTriggerThreshold_Name = CFSTR("Trigger Threshold");
static CFStringRef kTriggerResetDelay_Name = CFSTR("Trigger Reset Delay");
static CFStringRef kTriggerNote_Name = CFSTR("Trigger Note");
static CFStringRef kTriggerOctave_Name = CFSTR("Octave");
static CFStringRef kTriggerMidiChannel_Name = CFSTR("MIDI Channel");
static CFStringRef kTriggerMode_Name = CFSTR("Trigger Mode");
static CFStringRef kWaiting_Name = CFSTR("Waiting");
static CFStringRef kInternalMidi_Name = CFSTR("Internal Midi");
static CFStringRef kPlayState_Name = CFSTR("Play State");

const int kMinWindowSize = 0;
const int kMaxWindowSize = 99;
const int kDefaultWindowSize = 0;

const float kMinTriggerThreshold = 0;
const float kMaxTriggerThreshold = 1;
const float kDefaultTriggerThreshold = 1.0;

const int kMinTriggerResetDelay = 0;
const int kMaxTriggerResetDelay = 19;
const int kDefaultTriggerResetDelay = 0;

const int kMinTriggerNote = kNote_C;
const int kDefaultTriggerNote = kNote_E;
const int kMaxTriggerNote = kNote_B;

const int kMinOctave = -2;
const int kDefaultOctave = 2;
const int kMaxOctave = 8;

const int kMinMidiChannel = 0;
const int kDefaultMidiChannel = 1;
const int kMaxMidiChannel = 15;

const int kMinTriggerMode = 0;
const int kDefaultTriggerMode = kTriggerModeRms;
const int kMaxTriggerMode = 1;

const int WAITING_OFF = 0;
const int WAITING_ON = 1;
const int kMinWaiting = WAITING_OFF;
const int kDefaultWaiting = WAITING_OFF;
const int kMaxWaiting = WAITING_ON;

const int MIDI_EXTERNAL = 0;
const int MIDI_INTERNAL = 1;
const int kMinInternalMidi = MIDI_EXTERNAL;
const int kDefaultInternalMidi = MIDI_INTERNAL;
const int kMaxInternalMidi = MIDI_INTERNAL;

const int PLAY_STATE_STOP = 0;
const int PLAY_STATE_RUN = 1;
const int kMinPlayState = PLAY_STATE_STOP;
const int kDefaultPlayState = PLAY_STATE_STOP;
const int kMaxPlayState = PLAY_STATE_RUN;

// Factory presets
static const int kPreset_One = 0;
static const int kNumberPresets = 1;

static AUPreset kPresets[kNumberPresets] =
{
    { kPreset_One, CFSTR("Preset One") }
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#pragma mark ____Construction_Initialization

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTrigger::VirtualTrigger
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
VirtualTrigger::VirtualTrigger(AudioUnit component) : AUEffectBase(component) {
    
    
	// all the parameters must be set to their initial values here
	//
	// these calls have the effect both of defining the parameters for the first time
	// and assigning their initial values
	//
    SetParameter(kTriggerParam_WindowSize, kDefaultWindowSize);
    SetParameter(kTriggerParam_TriggerThreshold, kDefaultTriggerThreshold);
    SetParameter(kTriggerParam_TriggerResetDelay, kDefaultTriggerResetDelay);
    SetParameter(kTriggerParam_TriggerNote, kDefaultTriggerNote);
    SetParameter(kTriggerParam_TriggerOctave, kDefaultOctave);
    SetParameter(kTriggerParam_MidiChannel, kDefaultMidiChannel);
    SetParameter(kTriggerParam_TriggerMode, kDefaultTriggerMode);
    SetParameter(kTriggerParam_Waiting, kDefaultWaiting);
    SetParameter(kTriggerParam_InternalMidi, kDefaultInternalMidi);
    SetParameter(kTriggerParam_PlayState, kDefaultPlayState);
    
	SetParamHasSampleRateDependency(true);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTrigger::Initialize
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OSStatus VirtualTrigger::Initialize() {
    
    //create effect base
	OSStatus result = AUEffectBase::Initialize();
    
	if (result == noErr ) {
        
        //create MIDI client
        MIDIClientCreate(CFSTR("VirtualTrigger MIDI"), NULL, NULL, &mMidiClient);
        //create MIDI output endpoint
        MIDISourceCreate(mMidiClient, CFSTR("Virtual Trigger Endpoint"), &mMidiSource);
        //create output MIDI port
        result = MIDIOutputPortCreate(mMidiClient, CFSTR("VirtualTrigger MIDI Out Port"), &mMidiOutputPort);
            
        // in case the AU was un-initialized and parameters were changed, the view can now
        // be made aware it needs to update the view to reflect changes
        PropertyChanged(kAudioUnitCustomProperty_AudioDisplayData, kAudioUnitScope_Global, 0);
        
        mChannels = GetNumberOfChannels();
        mSampleRate = GetSampleRate();
        
        mWaiting = false;
        mWaitCount = 0;

        mAudioDisplayData.amplitude = 0;
        mAudioDisplayData.triggered = false;
	}
    
	return result;
}

void VirtualTrigger::Cleanup() {
    
    if (mMidiClient) MIDIClientDispose(mMidiClient);
    if (mMidiOutputPort) MIDIPortDispose(mMidiOutputPort);
}

OSStatus VirtualTrigger::Render(AudioUnitRenderActionFlags & ioActionFlags,
                                AudioTimeStamp const& inTimeStamp,
                                UInt32 inFramesToProcess) {
    
    UInt32 actionFlags = 0;
	OSStatus err = PullInput(0, actionFlags, inTimeStamp, inFramesToProcess);
	if (err) return err;
    
    //reset
    bool triggered = false;
    double audioSum = 0;
    MIDITimeStamp triggerTime = 0;
    
    //midi data
    char pktBuffer[1024];
    MIDIPacketList *pktList = (MIDIPacketList*) pktBuffer;
    
    //midi parameters
    int midiChannel = (int) GetParameter(kTriggerParam_MidiChannel);
    int triggerNote = (int) GetParameter(kTriggerParam_TriggerNote);
    int triggerOctave = (int) GetParameter(kTriggerParam_TriggerOctave);
    int internalMidi = (int) GetParameter(kTriggerParam_InternalMidi);

    //process audio buffers
    OSStatus result = AUEffectBase::Render(ioActionFlags, inTimeStamp, inFramesToProcess);
    
    //get data out of each channel
    for (int i = 0; i < mChannels; i++) {

        //get reference to kernel
        VirtualTriggerKernel *virtualTriggerKernel = dynamic_cast<VirtualTriggerKernel*>(mKernelList[i]);
        
        struct AudioProcessData audioProcessData = virtualTriggerKernel->GetAudioProcessData();
        
        //get kernel rms
        audioSum += audioProcessData.rms;
        
        //only update if not already triggered
        if (!triggered) {
            //get kernel trigger state trigger condition
            triggered = audioProcessData.triggered;
            
            //GetParameter(kAudioUnitProperty_Latency);
            triggerTime = audioProcessData.triggerTime;
            //inTimeStamp.mSampleTime;
        }
    }

    //populate display data
    //audio sum is across x channels
    //so get average
    mAudioDisplayData.amplitude = audioSum / mChannels;

    //deal with trigger and wait conditions
    
    //wait phase after trigger event
    if (mWaiting) {
        
        int waitDelay = GetParameter(kTriggerParam_TriggerResetDelay);

        //set display value to silence
        mAudioDisplayData.amplitude = 0;
        
        if (mWaitCount < waitDelay) {
            //carry on waiting
            mWaitCount++;
        } else {
            //finished waiting
            SetParameter(kTriggerParam_Waiting, WAITING_OFF);
        }
    }
    
    //did audio cause trigger
    if (triggered) {
        
        //MIDI note on
        Byte midiStatusByteOn = 144 + midiChannel; //magic 144 note on
        Byte dataNoteOn = triggerNote + (triggerOctave * 12) + 24;
        const Byte dataOn[3] = {midiStatusByteOn, dataNoteOn, 127}; //magic 127 max velocity
        
        //MIDI note off
        Byte midiStatusByteOff = 128 + midiChannel; //magic 144 note off
        Byte dataNoteOff = triggerNote + (triggerOctave * 12) + 24;
        const Byte dataOff[3] = {midiStatusByteOff, dataNoteOff, 0}; //magic 0 min velocity

        //Add MIDI events to packet list
        MIDIPacket *pkt = MIDIPacketListInit(pktList);
        //note on
        //magic 1024 packet size, 0 timestamp, 3 data size
        pkt = MIDIPacketListAdd(pktList, 1024, pkt, triggerTime, 3, dataOn);
        //note off
        MIDIPacketListAdd(pktList, 1024, pkt, triggerTime + 1000000000, 3, dataOff);
        
        if (internalMidi > 0) {
            //internal
            MIDIReceived(mMidiSource, pktList);
            
        } else {
            //external
            for (ItemCount i = 0; i < MIDIGetNumberOfDestinations(); ++i) {
                MIDIEndpointRef outputEndpoint = MIDIGetDestination(i);
                MIDISend(mMidiOutputPort, outputEndpoint, pktList);
            }
        }
        
        //set display to triggered
        mAudioDisplayData.triggered = true;
        //set display value to silence
        mAudioDisplayData.amplitude = 0;

        //stop retriggering in kernels
        SetParameter(kTriggerParam_Waiting, WAITING_ON);
    }
    
    //notify UI of new amplitude/triggered
    PropertyChanged(kAudioUnitCustomProperty_AudioDisplayData, kAudioUnitScope_Global, 0);

    return result;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#pragma mark ____Parameters

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTrigger::GetParameterInfo
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OSStatus VirtualTrigger::GetParameterInfo(AudioUnitScope inScope,
                                          AudioUnitParameterID inParameterID,
                                          AudioUnitParameterInfo &outParameterInfo) {
    
	OSStatus result = noErr;
    
	outParameterInfo.flags = kAudioUnitParameterFlag_IsWritable + kAudioUnitParameterFlag_IsReadable;
    
	if (inScope == kAudioUnitScope_Global) {
		
		switch(inParameterID) {
                
            case kTriggerParam_WindowSize:
				AUBase::FillInParameterName (outParameterInfo, kWindowSize_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_SampleFrames;
				outParameterInfo.minValue = kMinWindowSize;
				outParameterInfo.maxValue = kMaxWindowSize;
				outParameterInfo.defaultValue = kDefaultWindowSize;
				break;
                
            case kTriggerParam_TriggerThreshold:
				AUBase::FillInParameterName (outParameterInfo, kTriggerThreshold_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_Generic;
				outParameterInfo.minValue = kMinTriggerThreshold;
				outParameterInfo.maxValue = kMaxTriggerThreshold;
				outParameterInfo.defaultValue = kDefaultTriggerThreshold;
				break;
                
            case kTriggerParam_TriggerResetDelay:
				AUBase::FillInParameterName (outParameterInfo, kTriggerResetDelay_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_Generic;
				outParameterInfo.minValue = kMinTriggerResetDelay;
				outParameterInfo.maxValue = kMaxTriggerResetDelay;
				outParameterInfo.defaultValue = kDefaultTriggerResetDelay;
				break;
  
            case kTriggerParam_TriggerNote:
				AUBase::FillInParameterName (outParameterInfo, kTriggerNote_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_Generic;
				outParameterInfo.minValue = kMinTriggerNote;
				outParameterInfo.maxValue = kMaxTriggerNote;
				outParameterInfo.defaultValue = kDefaultTriggerNote;
				break;
                
            case kTriggerParam_TriggerOctave:
				AUBase::FillInParameterName (outParameterInfo, kTriggerOctave_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_Generic;
				outParameterInfo.minValue = kMinOctave;
				outParameterInfo.maxValue = kMaxOctave;
				outParameterInfo.defaultValue = kDefaultOctave;
				break;
                
            case kTriggerParam_MidiChannel:
				AUBase::FillInParameterName (outParameterInfo, kTriggerMidiChannel_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_Generic;
				outParameterInfo.minValue = kMinMidiChannel;
				outParameterInfo.maxValue = kMaxMidiChannel;
				outParameterInfo.defaultValue = kDefaultMidiChannel;
				break;
                
            case kTriggerParam_TriggerMode:
				AUBase::FillInParameterName (outParameterInfo, kTriggerMode_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_Generic;
				outParameterInfo.minValue = kMinTriggerMode;
				outParameterInfo.maxValue = kMaxTriggerMode;
				outParameterInfo.defaultValue = kDefaultTriggerMode;
				break;
                
            case kTriggerParam_Waiting:
                //read only
                outParameterInfo.flags = kAudioUnitParameterFlag_IsReadable;
                AUBase::FillInParameterName (outParameterInfo, kWaiting_Name, false);
				outParameterInfo.unit = kAudioUnitParameterUnit_Generic;
				outParameterInfo.minValue = kMinWaiting;
				outParameterInfo.maxValue = kMaxWaiting;
				outParameterInfo.defaultValue = kDefaultWaiting;
				break;

            case kTriggerParam_InternalMidi:
                AUBase::FillInParameterName (outParameterInfo, kInternalMidi_Name, false);
                outParameterInfo.unit = kAudioUnitParameterUnit_Boolean;
                outParameterInfo.minValue = kMinInternalMidi;
                outParameterInfo.maxValue = kMaxInternalMidi;
                outParameterInfo.defaultValue = kDefaultInternalMidi;
                break;

            case kTriggerParam_PlayState:
                //read only
                outParameterInfo.flags = kAudioUnitParameterFlag_IsReadable;
                AUBase::FillInParameterName (outParameterInfo, kPlayState_Name, false);
                outParameterInfo.unit = kAudioUnitParameterUnit_Boolean;
                outParameterInfo.minValue = kMinPlayState;
                outParameterInfo.maxValue = kMaxPlayState;
                outParameterInfo.defaultValue = kDefaultPlayState;
                break;

            default:
				result = kAudioUnitErr_InvalidParameter;
				break;
		}
	} else {
		result = kAudioUnitErr_InvalidParameter;
	}
	
	return result;
}

void VirtualTrigger::SetParameter(AudioUnitParameterID inID, AudioUnitParameterValue inValue) {

    if (inID == kTriggerParam_PlayState) {

        if (((float) inValue) == PLAY_STATE_STOP) {
            //reset kernel states
            for (int i = 0; i < mChannels; i++) {
                //get reference to kernel
                VirtualTriggerKernel *virtualTriggerKernel = dynamic_cast<VirtualTriggerKernel*>(mKernelList[i]);
                virtualTriggerKernel->Reset();
            }

        }

    } else if (inID == kTriggerParam_Waiting) {
        
        if (((float) inValue) == WAITING_OFF) {
            mWaiting = false;
        } else if (((float) inValue) == WAITING_ON) {
            mWaiting = true;
        }

        mWaitCount = 0;
    }
    
    AUEffectBase::SetParameter(inID, inValue);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#pragma mark ____Properties

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTrigger::GetPropertyInfo
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OSStatus VirtualTrigger::GetPropertyInfo (AudioUnitPropertyID inID,
                                          AudioUnitScope inScope,
                                          AudioUnitElement inElement,
                                          UInt32 & outDataSize,
                                          Boolean & outWritable) {
    
	if (inScope == kAudioUnitScope_Global) {
		
        switch (inID) {
                
			case kAudioUnitProperty_CocoaUI:
				outWritable = false;
				outDataSize = sizeof (AudioUnitCocoaViewInfo);
				return noErr;
                
            case kAudioUnitCustomProperty_AudioDisplayData:
                outWritable = false;
                outDataSize = sizeof(AudioDisplayData);
                return noErr;
        }
    }
	
	return AUEffectBase::GetPropertyInfo (inID, inScope, inElement, outDataSize, outWritable);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTrigger::GetProperty
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OSStatus VirtualTrigger::GetProperty (AudioUnitPropertyID inID,
                                      AudioUnitScope inScope,
                                      AudioUnitElement inElement,
                                      void * outData) {
    
	if (inScope == kAudioUnitScope_Global) {
        
		switch (inID) {
                
            // This property allows the host application to find the UI associated with this
            // AudioUnit
            //
			case kAudioUnitProperty_CocoaUI: {
                
				// Look for a resource in the main bundle by name and type.
				CFBundleRef bundle = CFBundleGetBundleWithIdentifier( CFSTR("com.shau.audiounit.VirtualTrigger") );
				
				if (bundle == NULL) return fnfErr;
                
				CFURLRef bundleURL = CFBundleCopyResourceURL( bundle,
                                                             CFSTR("CocoaTriggerView"),	// this is the name of the cocoa bundle as specified in the CocoaViewFactory.plist
                                                             CFSTR("bundle"),			// this is the extension of the cocoa bundle
                                                             NULL);
                
                if (bundleURL == NULL) return fnfErr;
                
				CFStringRef className = CFSTR("VirtualTrigger_ViewFactory");	// name of the main class that implements the AUCocoaUIBase protocol
				AudioUnitCocoaViewInfo cocoaInfo = { bundleURL, { className } };
				*((AudioUnitCocoaViewInfo *)outData) = cocoaInfo;
				
				return noErr;
			}
                
            case kAudioUnitCustomProperty_AudioDisplayData: {
                
                // the kernels are only created if we are initialized
				// the UI should check for the error and not draw
				if(!IsInitialized() ) return kAudioUnitErr_Uninitialized;
                
                //get and set UI values
				AudioDisplayData *audioDisplayData = ((AudioDisplayData *) outData);

                audioDisplayData->amplitude = mAudioDisplayData.amplitude;
                audioDisplayData->triggered = mAudioDisplayData.triggered;
                
                //reset trigger state
                mAudioDisplayData.triggered = false;
                
                return  noErr;
            }
        }
	}
	
	// if we've gotten this far, handles the standard properties
	return AUEffectBase::GetProperty(inID, inScope, inElement, outData);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#pragma mark ____Presets

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTrigger::GetPresets
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OSStatus VirtualTrigger::GetPresets (CFArrayRef * outData) const {
    
    // this is used to determine if presets are supported
    // which in this unit they are so we implement this method!
	if (outData == NULL) return noErr;
	
	CFMutableArrayRef theArray = CFArrayCreateMutable (NULL, kNumberPresets, NULL);
	for (int i = 0; i < kNumberPresets; ++i) {
		CFArrayAppendValue (theArray, &kPresets[i]);
    }
    
	*outData = (CFArrayRef)theArray;	// client is responsible for releasing the array
	return noErr;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTrigger::NewFactoryPresetSet
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OSStatus VirtualTrigger::NewFactoryPresetSet (const AUPreset & inNewFactoryPreset) {
    
	SInt32 chosenPreset = inNewFactoryPreset.presetNumber;
	
	for (int i = 0; i < kNumberPresets; ++i) {
        
		if (chosenPreset == kPresets[i].presetNumber) {
            
			// set whatever state you need to based on this preset's selection
			//
			// Here we use a switch statement, but it would also be possible to
			// use chosenPreset as an index into an array (if you publish the preset
			// numbers as indices in the GetPresets() method)
			//
			switch(chosenPreset) {
                    
				case kPreset_One:
                    SetParameter(kTriggerParam_WindowSize, 32);
                    SetParameter(kTriggerParam_TriggerThreshold, 0.5);
                    SetParameter(kTriggerParam_TriggerResetDelay, 5);
                    SetParameter(kTriggerParam_TriggerNote, kDefaultTriggerNote);
                    SetParameter(kTriggerParam_TriggerOctave, kDefaultOctave);
                    SetParameter(kTriggerParam_MidiChannel, kDefaultMidiChannel);
                    SetParameter(kTriggerParam_TriggerMode, kTriggerModeRms);
                    SetParameter(kTriggerParam_Waiting, kDefaultWaiting);
                    SetParameter(kTriggerParam_InternalMidi, kDefaultInternalMidi);
					break;
			}
            
            SetAFactoryPresetAsCurrent (kPresets[i]);
			return noErr;
		}
	}
	
	return kAudioUnitErr_InvalidPropertyValue;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#pragma mark ____VirtualTriggerKernel


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTriggerKernel::VirtualTriggerKernel()
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
VirtualTriggerKernel::VirtualTriggerKernel(AUEffectBase *inAudioUnit) : AUKernelBase(inAudioUnit) {
    
	Reset();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTriggerKernel::~VirtualTriggerKernel()
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
VirtualTriggerKernel::~VirtualTriggerKernel() {
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTriggerKernel::Reset()
//
//		It's very important to fully reset all state variables to their
//		initial settings here.  For delay/reverb effects, the delay buffers must
//		also be cleared here.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void VirtualTriggerKernel::Reset() {
    
    //initiliase state
    mAudioProcessData.rms = 0;
    mAudioProcessData.triggered = false;
    mAudioProcessData.triggerTime = 0;
    mAudioProcessData.triggerFrame = 0;
    
    mPreviousBufferSize = 0;
    mWindowEnergy = 0;
}

struct AudioProcessData VirtualTriggerKernel::GetAudioProcessData() {
    
    struct AudioProcessData result;
    
    result.rms = mAudioProcessData.rms;
    result.triggered = mAudioProcessData.triggered;
    result.triggerFrame = mAudioProcessData.triggerFrame;
    result.triggerTime = mAudioProcessData.triggerTime;

    //reset state
    mAudioProcessData.rms = 0;
    mAudioProcessData.triggered = false;
    mAudioProcessData.triggerTime = 0;
    mAudioProcessData.triggerFrame = 0;
    
    return result;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	VirtualTriggerKernel::Process(int inFramesToProcess)
//
//		We process one non-interleaved stream at a time
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void VirtualTriggerKernel::Process(	const Float32 *inSourceP,
                                   Float32 *inDestP,
                                   UInt32 inFramesToProcess,
                                   UInt32 inNumChannels,	// for version 2 AudioUnits inNumChannels is always 1
                                   bool &ioSilence) {
    
    double bufferEnergy = 0; //used for audio display data
    
    const Float32 *sourceP = inSourceP;
    
    int windowSize = (int) GetParameter(kTriggerParam_WindowSize);
    int triggerMode = (int) GetParameter(kTriggerParam_TriggerMode);
    int waiting = (int) GetParameter(kTriggerParam_Waiting);
    
    float threshold = GetParameter(kTriggerParam_TriggerThreshold);

    if (windowSize > 0) {
        //set minimum size for window
        windowSize += 10;
        //we are only spanning 1 window
        //max window size is 100 samples which is OK for 128, 256, 512 & 1024 sample buffers
        //however safety check for low latency 32 and 64 sample buffers
        if (windowSize > inFramesToProcess) {
            windowSize = inFramesToProcess;
        }
        
        //rebuild window energy buffer in the case that window size has changed
        mWindowEnergy = 0;
        for (int i = 0; i < windowSize; i++) {
            float audioData = mPreviousBuffer[mPreviousBufferSize - windowSize + i];
            mWindowEnergy += (audioData * audioData);
        }
    }
    
    //iterate through frames in buffer
    for (int i = 0; i < inFramesToProcess; i++) {
        
        //get head of buffer
        float head = sourceP[i];
        
        //display energy updated for each buffer pass
        bufferEnergy += (head * head);
        
        //only process RMS if window size available
        //run wether waiting, triggered OR peak
        if (windowSize == 0) {
            //no window reset window energy
            mWindowEnergy = 0;
            //and don't bother with RMS processing
        } else {
            
            //add RMS square to head of buffer
            mWindowEnergy += (head * head);
            
            //now get tail of buffer which may be in previous window
            //and take it away from audio energy calculation
            float tail = 0.0;
            int previousWindowPointerPosition = mPreviousBufferSize - windowSize + i;
            
            if (previousWindowPointerPosition > 0 && previousWindowPointerPosition < mPreviousBufferSize) {
                //get tail from previous window
                tail = mPreviousBuffer[previousWindowPointerPosition];
            } else {
                //tail value is in this window
                //check head has gone beyond window size
                if ((i - windowSize) >= 0) {
                    tail = sourceP[i - windowSize];
                }
            }
            
            //remove RMS square of tail from buffer
            mWindowEnergy -= (tail * tail);
        }
        
        //check for trigger conditions
        //depends on mode
        //but first check if we are in not in wait
        if (WAITING_OFF == waiting) {
            
            if (triggerMode == kTriggerModeRms) {
                    
                //RMS
                //check if threshold exceeded
                if (mWindowEnergy > 0) {
                    if (sqrtf(mWindowEnergy / windowSize)  > threshold) {
                        //trigger condition :)
                        mAudioProcessData.triggered = true;
                        mAudioProcessData.triggerTime = mach_absolute_time();
                        mAudioProcessData.triggerFrame = i;
                    }
                }
            } else if (triggerMode == kTriggerModePeak) {
                    
                //PEAK
                if (fabs(head) > threshold) {
                    //trigger condition :)
                    mAudioProcessData.triggered = true;
                    mAudioProcessData.triggerTime = mach_absolute_time();
                    mAudioProcessData.triggerFrame = i;
                }
            }
        }
    }

    //need prevous buffer for processing
    mPreviousBufferSize = inFramesToProcess;
    mPreviousBuffer = sourceP;

    //Calculate RMS for display
    mAudioProcessData.rms = sqrt(bufferEnergy / inFramesToProcess);
}