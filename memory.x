/* memory.x */
MEMORY
{
    /* RP2350 Flash starts at 0x10000000.
       We'll use 4MB as a safe default for most boards. */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4096K

    /* RP2350 has 520KB of SRAM starting at 0x20000000. */
    RAM   : ORIGIN = 0x20000000, LENGTH = 520K
}

/* Push code back to 0x10000200 so IMAGE_DEF can sit between the vector
   table (~0x10000110) and the start of .text without overlapping.
   The RP2350 boot ROM scans the first 4KB for the IMAGE_DEF block;
   without this it lands past offset 4096 and the ROM stays in BOOTSEL. */
_stext = 0x10000200;

SECTIONS {
    .start_block :
    {
        KEEP(*(.start_block));
    } > FLASH
} INSERT AFTER .vector_table;
