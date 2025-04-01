MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1M
    RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 320K
    SDRAM      : ORIGIN = 0xc0000000, LENGTH = 8M
}

SECTIONS
{
    .frame_buffer (NOLOAD) : {
        . = ALIGN(4);
        *(.frame_buffer);
        . = ALIGN(4);
    } > SDRAM

    .heap (NOLOAD) : {
        . = ALIGN(4);
        *(.heap);
        . = ALIGN(4);
    } > SDRAM
}
