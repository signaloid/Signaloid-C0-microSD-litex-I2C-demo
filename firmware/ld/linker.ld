INCLUDE output_format.ld
ENTRY(_start)

__DYNAMIC = 0;

INCLUDE regions.ld

SECTIONS
{
	.text :
	{
		_ftext = .;
		*(.text)
		*(.text .stub .text.* .gnu.linkonce.t.*)
		_etext = .;
	} > rom

	.rodata :
	{
		. = ALIGN(4);
		_fdata_rom = .;
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		*(.rodata1)
		*(.srodata .srodata.*)
		. = ALIGN(4);
		_edata_rom = .;
	} > rom

	.data : AT (ADDR(.rodata) + SIZEOF (.rodata))
	{
		. = ALIGN(4);
		_fdata = .;
		*(.data .data.* .gnu.linkonce.d.*)
		*(.data1)
		*(.ramtext .ramtext.*)
		_gp = ALIGN(16);
		*(.sdata .sdata.* .gnu.linkonce.s.* .sdata2 .sdata2.*)
		_edata = ALIGN(16);
	} > sram

	.bss :
	{
		. = ALIGN(4);
		_fbss = .;
		*(.dynsbss)
		*(.sbss .sbss.* .gnu.linkonce.sb.*)
		*(.scommon)
		*(.dynbss)
		*(.bss .bss.* .gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		_end = .;
	} > sram
}

PROVIDE(_fstack = ORIGIN(sram) + LENGTH(sram));
