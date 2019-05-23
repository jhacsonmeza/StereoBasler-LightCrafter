// LightCrafter4500Flash.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include "LC_Flash.h"

#include "dlpc350_common.h"
#include "dlpc350_usb.h"
#include "dlpc350_api.h"

#include <string>
#include <cstdio>
#include <string>
#include <sstream>


using namespace std;


int LightCrafterFlash(int exposurePeriod, int framePeriod, int repeat, string seq="all")
{

	unsigned char splashLut[64]; // Array where LUT entries to be sent are stored
	int numFlashImSeq; // Number of flash images in the sequence
	bool empty_seq{ true }; // Flag to know if a pattern sequence is parsed


	// Connect to device
	DLPC350_USB_Init();
	if (DLPC350_USB_IsConnected())
		DLPC350_USB_Close();

	DLPC350_USB_Open();
	if (!DLPC350_USB_IsConnected())
	{
		printf("Failed to open");
		return -1;
	}



	// Get the total number of images stored in flash memory
	unsigned int NumImgInFlash;
	DLPC350_GetNumImagesInFlash(&NumImgInFlash);


    // Constrcut image sequence
    if (seq == "all")
    {
        numFlashImSeq = NumImgInFlash;
        for (unsigned int i = 0; i < NumImgInFlash; i++)
            splashLut[i] = i;

    }
    else
    {
        istringstream fstr(seq);
        string s;
        numFlashImSeq = 0; // Get number of images in the given sequence

        while (getline(fstr, s, '-'))
        {
            if (NumImgInFlash - 1 < stoul(s, 0, 10))
            {
                printf("Image index error");
                return -1;
            }

            splashLut[numFlashImSeq++] = stoi(s);
        }
    }



	// Set display mode
	bool mode = true; // true: Pattern display mode, false: Video display mode

	if (DLPC350_SetMode(mode) < 0)
	{
		printf("Failed to set mode");
		return -1;
	}



	// Stop the current pattern sequence
	int action = 0; // 0 stop, 1 pause, 2 start

	if (DLPC350_PatternDisplay(action) < 0)
	{
		printf("Failed to set pattern display");
		return -1;
	}



	// Clear locally stored pattern LUT
	if (DLPC350_ClearPatLut() < 0)
	{
		printf("Failed to clear stored pattern LUT");
		return -1;
	}



	// Set up pattern LUT
	int trigType = 0; // 0 internal, 1 external positive, 2 external negative, 3 no input trigger
	int bitDepth = 8; // from 1 to 8
	int bitplaneGroups = 1; // Number of bitplane groups, for 8-bit depth there are 3 patterns to stream
	int patNum[] = { 0, 1, 2 }; // Index of the 3 bitplane groups (patterns) with 8 bitplanes each one
	int ledSelect = 7; // 0 no led, 1 red, 2 green, 3 yellow, 4 blue, 5 magenta, 6 cyan, 7 white
	bool invertPat = false; // true: invert pattern, false: do not invert pattern
	bool insertBlack = false; // true: insert black-fill pattern, false: do not do it
	bool bufSwap = false; // true: perform a buffer swap, false: do not do it
	bool trigOutPrev = false; // true: Trigger Out 1 will continue to be high.
							// false: Trigger Out 1 has a rising edge at the start of a pattern, 
							// and a falling edge at the end of the pattern
	int result;
	for (int i = 0; i < numFlashImSeq; i++)
	{
		for (int j = 0; j < bitplaneGroups; j++)
		{
			if (j == 0)
				result = DLPC350_AddToPatLut(trigType, patNum[j], bitDepth, ledSelect, invertPat, insertBlack, !bufSwap, trigOutPrev);
			else
				result = DLPC350_AddToPatLut(trigType, patNum[j], bitDepth, ledSelect, invertPat, insertBlack, bufSwap, trigOutPrev);

			if (result < 0)
			{
				printf("Failed to add to pattern LUT");
				return -1;
			}
		}
	}



	// Set pattern display data source
	bool external = false; // true: patterns from RGB/FPD-link interface, false: patterns from flash memory

	if (DLPC350_SetPatternDisplayMode(external) < 0)
	{
		printf("Failed to set pattern display mode");
		return -1;
	}



	// Set the sequence parameters
	unsigned int numPatsForTrigOut2; // Number of patterns to display

	numPatsForTrigOut2 = repeat ? 1 : numFlashImSeq * bitplaneGroups; // If repeat, variable must be set to 1, else, must be number of LUT patterns

	if (DLPC350_SetPatternConfig(numFlashImSeq*bitplaneGroups, repeat, numPatsForTrigOut2, numFlashImSeq) < 0)
	{
		printf("Failed to set pattern configuration");
		return -1;
	}



	// Set exposure time and frame period
	if (DLPC350_SetExposure_FramePeriod(exposurePeriod, framePeriod) < 0)
	{
		printf("Failed to set exposure/frame period");
		return -1;
	}



	// Set the pattern trigger mode
	int trigMode = 1; // 0 VSYNC serves to trigger the pattern display sequence
					// 1 Internally or Externally (through TRIG_IN1 and TRIG_IN2) generated trigger
					// 2 TRIG_IN_1 alternates between two patterns,while TRIG_IN_2 advances to the next pair of patterns
					// 3 Internally or externally generated trigger for Variable Exposure display sequence
					// 4 VSYNC triggered for Variable Exposure display sequence

	if (DLPC350_SetPatternTriggerMode(trigMode) < 0)
	{
		printf("Failed to set pattern trigger mode");
		return -1;
	}



	// Send pattern LUT to device
	if (DLPC350_SendPatLut() < 0)
	{
		printf("Failed to send pattern LUT");
		return -1;
	}



	// Send image LUT to device
	if (DLPC350_SendImageLut(&splashLut[0], numFlashImSeq) < 0)
	{
		printf("Failed to send image LUT");
		return -1;
	}



	// Validate the pattern LUT
	unsigned int status;

	if (DLPC350_ValidatePatLutData(&status) < 0)
	{
		printf("Failed to validate pattern LUT data");
		return -1;
	}



	// Start the pattern sequence
	action = 2; // 0 stop, 1 pause, 2 start
	if (DLPC350_PatternDisplay(action) < 0)
	{
		printf("Failed to set pattern display");
		return -1;
	}

	return 0;
}