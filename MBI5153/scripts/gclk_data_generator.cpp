#include <bitset>
#include <iostream>

/******** GCLK Calculation Bit Pulse Length and Data - ChatGPT generated **********/
/* LLM Prompt

Task Description:
The goal is to generate and print a series of byte sequences based on the following specifications:

Each sequence consists of 512 repeats of three bytes.
In each repeat, the first two bytes are the same and the third byte has its Most Significant Bit (MSB) set to 1.
Repeat the sequence generation 20 times, embedding the current repeat count (from 0 to 19) in the least significant bits (LSBs) of each byte in the sequence.
Add 20 zero bytes before and after each sequence, with also has the repeat value in the LSBs.
Instead of pre-allocating memory for the entire sequence, the byte value at a given position should be calculated on the fly.
*/

const int BYTES_PER_REPEAT  = 3; // gclk pulse on 3rd byte, gives us enough time to send frame data then.
const int REPEATS           = 513; // 513 gclks per the documentation
const int SEQUENCE_SIZE     = BYTES_PER_REPEAT * REPEATS;
const int REPEAT_COUNT      = 20; // 20 rows
const int PADDING_SIZE      = 8; // some delay between row changes?
const int TOTAL_SIZE        = (PADDING_SIZE + SEQUENCE_SIZE + PADDING_SIZE) * REPEAT_COUNT;


// Generate MBI5153 GCLK DATA for Byte 0 of parallel output.
// Function to calculate the value of a byte at a specific position
uint8_t getByteValue(int position) {
    int sequenceIndex = position / (PADDING_SIZE + SEQUENCE_SIZE + PADDING_SIZE);
    int offset = position % (PADDING_SIZE + SEQUENCE_SIZE + PADDING_SIZE);
/*
    // Handle padding
    if (offset < PADDING_SIZE || offset >= PADDING_SIZE + SEQUENCE_SIZE) {
        return 0x00;
    }
*/
    // Calculate position within the sequence
    int sequencePosition = offset - PADDING_SIZE;
    int byteIndex = sequencePosition % BYTES_PER_REPEAT;

    uint8_t repeatValue = sequenceIndex & 0x1F;  // Repeat value embedded in the LSBs

    // Handle padding
    if (offset < PADDING_SIZE || offset >= PADDING_SIZE + SEQUENCE_SIZE) {
        return repeatValue;
    }    

    // Calculate the byte value based on its position in the repeat
    if (byteIndex == 2) {
        return 0x80 | repeatValue;  // Third byte with MSB set
    } else {
        return repeatValue;  // First and second bytes
    }
}


// Function to print the sequence in binary format for verification
void printSequence() {
    for (int i = 0; i < TOTAL_SIZE; ++i) {
        std::cout << std::bitset<8>(getByteValue(i)) << " ";
        if ((i + 1) % 8 == 0) std::cout << std::endl;
    }
}

int main() {
    // Print the final sequence
    printSequence();

    return 0;
}

