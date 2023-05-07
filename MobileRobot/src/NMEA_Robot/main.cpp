#include <Arduino.h>
#include "drive.h"
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define DEBUG false

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo liftMotor;

void setup()
{
    Serial.begin(115200);
    driveSetup();
    // wirelessSetup();
    /* Initialise the sensor */

    liftMotor.attach(13);

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    delay(1000);

    bno.setExtCrystalUse(true);
}

void debugPrint(String debugText)
{
    if (DEBUG)
    {
        Serial.println(debugText);
    }
}

char inputData[100] = "";
char controlKeyword[] = "$CTRL"; //"$CTRL desiredMovementHeading, desiredOrientation, speed"
char liftKeyword[] = "$LFT";     //"$CTRL speed"

long millisLastDataUpdate = millis();
long millisLastLiftUpdate = millis();
float currentHeading;
float orientationGain = 3 / 180.0;
float desiredMovementHeading = 0;
float desiredOrientation = 0;
float desiredSpeed = 0;
float desriedLift = 0;

void loop()
{
    sensors_event_t event;
    bno.getEvent(&event);
    /// Collection
    if (Serial.available() > 0) // If there's new data to parse
    {
        size_t byteCount = Serial.readBytesUntil('\n', inputData, sizeof(inputData) - 1); // Read the Data into the buffer
        inputData[byteCount] = NULL;                                                      // End Character
        Serial.println(inputData);

        /// Parsing
        char *controlKeywordPointer = strstr(inputData, controlKeyword);
        char *liftKeywordPointer = strstr(inputData, liftKeyword);
        if (controlKeywordPointer == NULL)
        {
            // Start looking for lift
            if (liftKeywordPointer == NULL)
            {
                Serial.println("Couldn't find keyword");
            }
            else
            {

                int dataPosition = (liftKeywordPointer - inputData) + strlen(liftKeyword);
                const char delimiter[] = ",";
                char liftCommand[10];
                char *Lifttoken = strtok(&inputData[dataPosition], delimiter);

                if (Lifttoken != NULL && strlen(Lifttoken) < sizeof(liftCommand))
                {
                    strncpy(liftCommand, Lifttoken, sizeof(liftCommand));
                }
                desriedLift = atof(liftCommand);
                Serial.println(desriedLift);
                millisLastLiftUpdate = millis();
            }
        }
        else
        {
            // Process $CTRL
            int dataPosition = (controlKeywordPointer - inputData) + strlen(controlKeyword);

            const char delimiter[] = ",";
            char parsedCommands[3][20];
            // parsedCommands[0] = NULL;  // Other fixes that don't work. Don't have time to fix.
            // parsedCommands[1] = "\n";
            // parsedCommands[2] = "\n";
            char *token = strtok(&inputData[dataPosition], delimiter);
            int dataCount = 0;

            if (token != NULL && strlen(token) < sizeof(parsedCommands[0]))
            {
                strncpy(parsedCommands[0], token, sizeof(parsedCommands[0]));
                dataCount++;
            }
            else
            {
                Serial.println("Token too big.");
                strncpy(parsedCommands[0], "0", sizeof(parsedCommands[0]));
            }

            // Loop to get the remainder. (Difference is that strtok gets a null pointer to resume from previous)

            for (int i = 1; i < 3; i++)
            {
                // Serial.println(i);
                char *token = strtok(NULL, delimiter);
                if (token != NULL && strlen(token) < sizeof(parsedCommands[i]))
                {
                    strncpy(parsedCommands[i], token, sizeof(parsedCommands[i]));
                    dataCount++;
                }
                else
                {
                    Serial.println("Token too big.");
                    // strcpy(parsedCommands[i], NULL);  // Uncommenting causes a problem I don't have time to fix right now.
                    strncpy(parsedCommands[i], "0", sizeof(parsedCommands[i])); // Fix?
                }
            }
            if (DEBUG)
            {
                Serial.print("Found ");
                Serial.print(dataCount);
                Serial.println(" strings:");
                for (int i = 0; i < 3; i++)
                {
                    Serial.println(parsedCommands[i]);
                }
            }

            /// Converting Strings to floats.
            if (dataCount != 3)
            {
                Serial.println("Not enough data entered.");
            }
            else
            {
                desiredMovementHeading = atof(parsedCommands[0]);
                desiredOrientation = atof(parsedCommands[1]);
                desiredSpeed = atof(parsedCommands[2]);
                millisLastDataUpdate = millis();
            }
            Serial.println(desiredMovementHeading);
            Serial.println(desiredOrientation);
            Serial.println(desiredSpeed);
        }
    }

    /// Process assigning motor values.
    // Get compass value.
    // currentHeading = something somthing compass.get();
    currentHeading = -event.magnetic.x;

    Serial.print("$STATE,");
    Serial.print(currentHeading);
    Serial.println(",*");

    // Serial.println(millis() - millisLastDataUpdate);
    if (millis() - millisLastLiftUpdate > 1000)
    {
        liftMotor.write(0);
        // Serial.println("Zero lift");
    }
    else
    {
        liftMotor.write(desriedLift);
        // Serial.println("Setting lift");
    }
    if (millis() - millisLastDataUpdate > 1000)
    {
        // Set motors to 0 no matter what
        drive(0, 0, 0, 0);
    }
    else
    { // Process the motors.
        // Find orientation heading error
        float leftHandError = desiredOrientation - currentHeading;
        if (desiredOrientation < currentHeading)
        {
            leftHandError = leftHandError + 360;
        }
        float rightHandError = -desiredOrientation + currentHeading;
        if (desiredOrientation > currentHeading)
        {
            rightHandError = rightHandError + 360;
        }
        float headingError = 0;
        // Serial.print("LH ERROR: ");
        // Serial.println(leftHandError);
        // Serial.print("RH ERROR: ");
        // Serial.println(rightHandError);
        if (leftHandError < rightHandError)
        {
            headingError = leftHandError;
        }
        else
        {
            headingError = -rightHandError;
        }

        while (headingError >= 360)
        {
            headingError - 360;
        }
        while (headingError <= -360)
        {
            headingError + 360;
        }

        // Serial.print("Heading error: ");
        // Serial.println(headingError);

        // Find movement heading error
        // float leftHandedMovementHeadingAngle = desiredMovementHeading - currentHeading;  //Uncomment for global movement heading
        float leftHandedMovementHeadingAngle = desiredMovementHeading - 0; // Relative movement heading
        if (desiredMovementHeading < currentHeading)
        {
            leftHandedMovementHeadingAngle = leftHandedMovementHeadingAngle + 360;
        }
        // Serial.print("Movement Heading Error: ");
        // Serial.println(leftHandedMovementHeadingAngle);
        float straightComponent = cos(leftHandedMovementHeadingAngle * DEG_TO_RAD);
        float leftComponent = sin(leftHandedMovementHeadingAngle * DEG_TO_RAD);
        // Serial.println(orientationGain);
        // Serial.println(headingError * -orientationGain);

        // float frontLeft = (straightComponent + leftComponent) * desiredSpeed + headingError * -orientationGain;
        // float frontRight = (straightComponent - leftComponent) * desiredSpeed + headingError * orientationGain;
        // float backLeft = (straightComponent - leftComponent) * desiredSpeed + headingError * -orientationGain;
        // float backRight = (straightComponent + leftComponent) * desiredSpeed + headingError * orientationGain;
        float frontLeft = (straightComponent - leftComponent);
        float frontRight = (straightComponent + leftComponent);
        float backLeft = (straightComponent + leftComponent);
        float backRight = (straightComponent - leftComponent);

        float maximum = max(max(abs(frontLeft), abs(frontRight)), max(abs(backLeft), abs(backRight)));

        if (maximum != 0)
        {
            frontLeft = frontLeft / maximum * desiredSpeed;
            frontRight = frontRight / maximum * desiredSpeed;
            backLeft = backLeft / maximum * desiredSpeed;
            backRight = backRight / maximum * desiredSpeed;
        }

        // Serial.println("FRST--------");
        // Serial.println(frontLeft);
        // Serial.println(frontRight);
        // Serial.println(backLeft);
        // Serial.println(backRight);
        float maxOrietnation = 0.2;
        frontLeft = (frontLeft + constrain(headingError * -orientationGain, -maxOrietnation, maxOrietnation));
        frontRight = (frontRight + constrain(headingError * orientationGain, -maxOrietnation, maxOrietnation));
        backLeft = (backLeft + constrain(headingError * -orientationGain, -maxOrietnation, maxOrietnation));
        backRight = (backRight + constrain(headingError * orientationGain, -maxOrietnation, maxOrietnation));
        // Serial.println(frontLeft);
        // Serial.println(frontRight);
        // Serial.println(backLeft);
        // Serial.println(backRight);

        maximum = max(max(abs(frontLeft), abs(frontRight)), max(abs(backLeft), abs(backRight)));
        // Serial.println(maximum);
        if (maximum > 1)
        {
            frontLeft = frontLeft / maximum;
            frontRight = frontRight / maximum;
            backLeft = backLeft / maximum;
            backRight = backRight / maximum;
        }

        // if (DEBUG)
        // {
        // Serial.println("--------");
        // Serial.println(frontLeft);
        // Serial.println(frontRight);
        // Serial.println(backLeft);
        // Serial.println(backRight);
        // }
        drive(frontLeft, backLeft, frontRight, backRight);
    }
}
