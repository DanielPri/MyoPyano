// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.

#include "..\include\myo\myo.hpp"
#include "../include/irrKlang/irrKlang.h"

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
		:roll_w(0), pitch_w(0), yaw_w(0), origin_roll(0), origin_pitch(0), origin_yaw(0), currentPose(), whichArm(0)
    {
		roll_w = { 0, 0 };
		pitch_w = { 0, 0 };
		yaw_w = { 0, 0 };
		origin_roll = { 0, 0 };
		origin_pitch = { 0, 0 };
		origin_yaw = { 0, 0 };
		onArm[0] = false;
		onArm[1] = false;
		isUnlocked[0] = false;
		isUnlocked[1] = false;
		currentPose = { myo::Pose::unknown, myo::Pose::unknown };
		whichArm = { myo::armUnknown, myo::armUnknown };
    }

	void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
	{
		// Print out the MAC address of the armband we paired with.

		// The pointer address we get for a Myo is unique - in other words, it's safe to compare two Myo pointers to
		// see if they're referring to the same Myo.

		// Add the Myo pointer to our list of known Myo devices. This list is used to implement identifyMyo() below so
		// that we can give each Myo a nice short identifier.
		knownMyos.push_back(myo);

		// Now that we've added it to our list, get our short ID for it and print it out.
		std::cout << "Paired with " << identifyMyo(myo) << "." << std::endl;
	}

    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = {0, 0};
        pitch_w = { 0, 0 };
        yaw_w = { 0, 0 };
		origin_roll = { 0, 0 };
		origin_pitch = { 0, 0 };
		origin_yaw = { 0, 0 };
		onArm[0] = false;
		onArm[1] = false;
		isUnlocked[0] = false;
		isUnlocked[1] = false;
    }

    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;

		int myoIndex = identifyMyo(myo);

		//std::cout << "w: " << quat.w() << " x: " << quat.x() << " y: " << quat.y() << " z: " << quat.z() << "\n";
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
		
		if (origin_pitch[myoIndex] == 0)
		{
			origin_roll[myoIndex] = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 359);
			origin_pitch[myoIndex] = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 359);
			origin_yaw[myoIndex] = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 359);
		}

        // Convert the floating point angles in radians to a scale from 0 to 18.
        roll_w[myoIndex] = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 359);
        pitch_w[myoIndex] = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 359);
        yaw_w[myoIndex] = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 359);
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
		int myoIndex = identifyMyo(myo);
        currentPose[myoIndex] = pose;

        /*if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);

            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
        } else {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            myo->unlock(myo::Myo::unlockTimed);
        }*/
		std::cout << pose;
		if (pose == myo::Pose::fist)
		{
			std::cout << "fist";
			origin_pitch[myoIndex] = 0;
			origin_roll[myoIndex] = 0;
			origin_yaw[myoIndex] = 0;
		}
    }

    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
		int myoIndex = identifyMyo(myo);
        onArm[myoIndex] = true;
        whichArm[myoIndex] = arm;
    }

    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
		int myoIndex = identifyMyo(myo);
        onArm[myoIndex] = false;
    }

    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
		int myoIndex = identifyMyo(myo);
        isUnlocked[myoIndex] = true;
    }

    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
		int myoIndex = identifyMyo(myo);
        isUnlocked[myoIndex] = false;
    }

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
		for (int i = 0; i < knownMyos.size(); i++)
		{
			// Clear the current line
			std::cout << '\n';

			// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
			/*std::cout << "[ Roll: " << std::roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
			<< "[ Pitch: " << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
			<< "[ Yaw : " << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
			*/
			std::cout << "[ Roll: " << roll_w[i] << "] [ Pitch: " << pitch_w[i] << " ] [ Yaw: " << yaw_w[i] << " ]";
		
			if (onArm[i]){
				// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

				// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
				// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
				// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
				std::string poseString = currentPose[i].toString();

				std::cout << '[' << (isUnlocked[i] ? "unlocked" : "locked  ") << ']'
					<< '[' << (whichArm[i] == myo::armLeft ? "L" : "R") << ']'
					<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
			}
			else {
				// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
				std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
			}
		}

        std::cout << std::flush;
    }

	size_t identifyMyo(myo::Myo* myo) {
		// Walk through the list of Myo devices that we've seen pairing events for.
		for (size_t i = 0; i < knownMyos.size(); i++) {
			// If two Myo pointers compare equal, they refer to the same Myo device.
			if (knownMyos[i] == myo) {
				return i;
			}
		}

		return 0;
	}

	// We store each Myo pointer that we pair with in this list, so that we can keep track of the order we've seen
	// each Myo and give it a unique short identifier (see onPair() and identifyMyo() above).
	std::vector<myo::Myo*> knownMyos;

    // These values are set by onArmSync() and onArmUnsync() above.
	bool onArm[2];
	std::vector<myo::Arm> whichArm;

    // This is set by onUnlocked() and onLocked() above.
	bool isUnlocked[2];

    // These values are set by onOrientationData() and onPose() above.
	std::vector<int> roll_w, pitch_w, yaw_w;
	std::vector<int> origin_roll, origin_pitch, origin_yaw;
	std::vector<myo::Pose> currentPose;
};

int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {

	// start the sound engine with default parameters
	irrklang::ISoundEngine* engine = irrklang::createIrrKlangDevice();
	char* sounds[2] = { "Sounds/a.wav", "Sounds/b.wav"};

	if (!engine)
		return 0; // error starting up the engine

    // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
    // publishing your application. The Hub provides access to one or more Myos.
    myo::Hub hub("com.Pyano.MyoPyano");

    std::cout << "Attempting to find a Myo..." << std::endl;

    // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
    // immediately.
    // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
    // if that fails, the function will return a null pointer.
    //myo::Myo* myo = hub.waitForMyo(10000);

    // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
   /* if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }*/

    // We've found a Myo.
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);
	bool boolean = true;

	
	//engine->stopAllSounds();
	//engine->drop(); // delete engine

	
	//collector.currentPose();
	bool allowedSound[2] = { false, false };
    // Finally we enter our main loop.
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/20);
        // After processing events, we call the print() member function we defined above to print out the values we've
        // obtained from any events that have occurred.
		if (boolean) {
			collector.print();
			boolean = false;
		}
		
		for (int i = 0; i < collector.knownMyos.size(); i++)
		{
			if (collector.pitch_w[i] - collector.origin_pitch[i] > 50) {
				allowedSound[i] = true;
			}
			if (collector.pitch_w[i] - collector.origin_pitch[i] < 50 && allowedSound[i]) {
				//std::cout << "Wassup bitches!!\n";
				allowedSound[i] = false;
				// play some sound stream, not looped
				irrklang::ISound *sound = engine->play2D(sounds[i], false, false);
				if (sound)
					sound->drop();
			}
			/*
					if (collector.pitch_w > 180 && collector.pitch_w < 220) {
						std::cout << "Wassup bitches!!\n";
					}*/
		}
    }

	engine->drop();

    // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
