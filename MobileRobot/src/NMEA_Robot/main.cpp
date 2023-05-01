#include <Arduino.h>
#include "encoder.h"
#include "drive.h"

#define DEBUG false

void setup()
{
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    // wirelessSetup();
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

long millisLastDataUpdate = millis();
float currentHeading;
float orientationGain = 0.5 / 180.0;
float desiredMovementHeading = 0;
float desiredOrientation = 0;
float desiredSpeed = 0;

void loop()
{
    /// Collection
    if (Serial.available() > 0) // If there's new data to parse
    {
        size_t byteCount = Serial.readBytesUntil('\n', inputData, sizeof(inputData) - 1); // Read the Data into the buffer
        inputData[byteCount] = NULL;                                                      // End Character
        Serial.println(inputData);

        /// Parsing
        char *controlKeywordPointer = strstr(inputData, controlKeyword);
        if (controlKeywordPointer == NULL)
        {
            Serial.println("Couldn't find control keyword.");
        }
        else
        {
            int dataPosition = (controlKeywordPointer - inputData) + strlen(controlKeyword);

            const char delimiter[] = ",";
            char parsedCommands[3][20];
            // parsedCommands[0] = NULL;  // Other fixes that don't work. Don't have time to fix.
            // parsedCommands[1] = "\n";
            // parsedCommands[2] = "\n";
            int dataCount = 0;

            char *token = strtok(&inputData[dataPosition], delimiter);

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
    currentHeading = 0;
    // Serial.println(millis() - millisLastDataUpdate);
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
        float leftHandedMovementHeadingAngle = desiredMovementHeading - currentHeading;
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

        // Serial.println("FIRST--------");
        // Serial.println(frontLeft);
        // Serial.println(frontRight);
        // Serial.println(backLeft);
        // Serial.println(backRight);
        frontLeft = (frontLeft + headingError * -orientationGain);
        frontRight = (frontRight + headingError * orientationGain);
        backLeft = (backLeft + headingError * -orientationGain);
        backRight = (backRight + headingError * orientationGain);

        maximum = max(max(abs(frontLeft), abs(frontRight)), max(abs(backLeft), abs(backRight)));

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