#ifndef BIT_LIST_H
#define BIT_LIST_H

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <algorithm>

class BitList {
public:
    BitList(size_t bitCount): size((bitCount + 7) / 8), currentIndex(0), isValid(true) {
        bits = static_cast<uint8_t*>(malloc(size));
        if (!bits) {
            isValid = false;  // Set a flag if allocation fails
            return;
        }
        std::fill(bits, bits + size, 0);
    }

    ~BitList() {
        if (bits) free(bits);
    }

    bool addBit(bool bit) {
        if (!isValid) return false;  // Check if allocation was successful

        if (bit) {
            bits[currentIndex / 8] |= (1 << (currentIndex % 8));
        } else {
            bits[currentIndex / 8] &= ~(1 << (currentIndex % 8));
        }
        
        currentIndex = (currentIndex + 1) % (size * 8);
        return true;
    }

    bool getBit(size_t index) const {
        if (!isValid || index >= size * 8) return false;  // Check for valid access
        return bits[index / 8] & (1 << (index % 8));
    }

    size_t getSize() const {
        return isValid ? size * 8 : 0;
    }

    size_t countBits() const {
        if (!isValid) return 0;

        size_t count = 0;
        for (size_t i = 0; i < size; ++i) {
            count += __builtin_popcount(bits[i]);
        }
        return count;
    }

    void printBits() const {
        if (!isValid) {
            printf("Error: Invalid BitList\n");
            return;
        }

        printf("Bits: ");
        for (size_t i = 0; i < size; ++i) {
            for (size_t j = 0; j < 8; ++j) {
                printf("%d", (bits[i] & (1 << j)) != 0);
            }
        }
        printf("\n");
    }

    void clear() {
        if (isValid) {
            std::fill(bits, bits + size, 0);
            currentIndex = 0;
        }
    }

    bool isValidBitList() const {
        return isValid;
    }

private:
    size_t size;
    uint8_t* bits;
    size_t currentIndex;
    bool isValid;  // New flag to check allocation status
};

#endif // BIT_LIST_H
