-- Configuration file for the LM4F microcontroller

addi( sf( 'src/platform/%s/inc', platform ) )
addi( sf( 'src/platform/%s/driverlib', platform ) )

-- The microcontroller LM4F120H5QR supports USB CDC
addi( sf( 'src/platform/%s/usblib', platform ) )
addi( sf( 'src/platform/%s/usblib/device', platform ) )

specific_files = "startup_gcc.c platform.c platform_int.c lm4f_pio.c"
local fwlib_files = utils.get_files( "src/platform/" .. platform .. "/driverlib", ".*%.c$" )

-- The microcontroller LM4F120H5QR supports USB CDC
fwlib_files = fwlib_files .. " " .. utils.get_files( "src/platform/" .. platform .. "/usblib", ".*%.c$" ) 
fwlib_files = fwlib_files .. " " .. utils.get_files( "src/platform/" .. platform .. "/usblib/device", ".*%.c$" )
specific_files = specific_files .. "  usb_serial_structs.c"

ldscript = "lm4f.ld"

-- Prepend with path
specific_files = fwlib_files .. " " .. utils.prepend_path( specific_files, "src/platform/" .. platform )
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = sf( "src/platform/%s/%s", platform, ldscript )

addm{ "FOR" .. comp.cpu:upper(), 'gcc', 'CORTEX_M4' }

-- Standard GCC flags
addcf{ '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall' }
addlf{ '-nostartfiles', '-nostdlib', '-T', ldscript, '-Wl,--gc-sections', '-Wl,--allow-multiple-definition' }
addaf{ '-x', 'assembler-with-cpp', '-Wall' }
addlib{ 'c','gcc','m' }

-- TODO: Consider using thumb2 instruction set. May need to deal with src/platform/ arm files.
local target_flags =  {'-mcpu=cortex-m4','-mthumb' }

-- Configure general flags for target
addcf{ target_flags, '-mlittle-endian' }
addlf{ target_flags, '-Wl,-e,ResetISR', '-Wl,-static' }
addaf( target_flags )

-- Toolset data
tools.lm4f = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.lm4f.prog_flist = { output .. ".bin" }

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

