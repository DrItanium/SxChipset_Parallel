This is a modification of the standard SxChipset where ports C and A are reserved for parallel data access. Based off of the work I did with the management engine and an adafruit grand central m4 based chipset, I was able to prove that the biggest bottleneck in the chipset is actually serializing the data lines. The address lines are second but they are not used as much.
This version of the chipset also uses the 4809 based management engine firmware first used in the Grand Central M4 based chipset.

I have had to use other tricks to get everything to work and this is the just the beginning :).