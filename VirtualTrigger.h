// Here we define a custom property so the view is able to retrieve audio display and triggered values

// custom properties id's must be 64000 or greater
// see <AudioUnit/AudioUnitProperties.h> for a list of Apple-defined standard properties
//

enum
{
	kAudioUnitCustomProperty_AudioDisplayData = 65598
};

//audio display structure
typedef struct AudioDisplayData {
	double amplitude;
	bool triggered;
} AudioDisplayData;

//audio processing data
typedef struct AudioProcessData {
    float rms;
    bool triggered;
    uint64_t triggerTime;
    int triggerFrame;
} AudioProcessData;
