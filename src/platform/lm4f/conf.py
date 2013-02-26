# Configuration file for the LM4F microcontroller
import fnmatch
import glob
import os

comp.Append(CPPPATH = ['src/platform/%s/inc' % platform])
comp.Append(CPPPATH = ['src/platform/%s/driverlib' % platform])

# The microcontroller LM4F120H5QR supports USB CDC
comp.Append(CPPPATH = ['src/platform/%s/usblib' % platform])
comp.Append(CPPPATH = ['src/platform/%s/usblib/device' % platform])

fwlib_files = " ".join(glob.glob("src/platform/%s/driverlib/*.c" % platform))

specific_files = "startup_gcc.c platform.c platform_int.c lm4f_pio.c"

# The microcontroller LM4F120H5QR supports USB CDC
fwlib_files = fwlib_files + " " + " ".join(glob.glob("src/platform/%s/usblib/*.c" % platform))
fwlib_files = fwlib_files + " " + " ".join(glob.glob("src/platform/%s/usblib/device/*.c" % platform))
specific_files = specific_files + " usb_serial_structs.c"

ldscript = "lm4f.ld"

# Prepend with path
specific_files = fwlib_files + " " + " ".join( [ "src/platform/%s/%s" % ( platform, f ) for f in specific_files.split() ] )
specific_files += " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = "src/platform/%s/%s" % ( platform, ldscript )

comp.Append(CPPDEFINES = ["FOR" + comp[ 'cpu' ],'gcc'])
comp.Append(CPPDEFINES = ['CORTEX_M4'])

# TODO: check if we need to pass a flag to FP being active
# Standard GCC Flags
comp.Append(CCFLAGS = ['-ffunction-sections','-fdata-sections','-fno-strict-aliasing','-Wall'])
comp.Append(LINKFLAGS = ['-nostartfiles','-nostdlib','-T',ldscript,'-Wl,--gc-sections','-Wl,--allow-multiple-definition'])
comp.Append(ASFLAGS = ['-x','assembler-with-cpp','-c','-Wall','$_CPPDEFFLAGS'])
comp.Append(LIBS = ['c','gcc','m'])

# TODO: Consider using thumb2 instruction set. May need to deal with src/platform/ arm files.
TARGET_FLAGS = ['-mcpu=cortex-m4','-mthumb']

# Configure General Flags for Target
comp.Prepend(CCFLAGS = [TARGET_FLAGS,'-mlittle-endian'])
comp.Prepend(LINKFLAGS = [TARGET_FLAGS,'-Wl,-e,ResetISR','-Wl,-static'])
comp.Prepend(ASFLAGS = TARGET_FLAGS)

# Toolset data
tools[ 'lm4f' ] = {}

# Programming function
def progfunc_lm4f( target, source, env ):
  outname = output + ".elf"
  os.system( "%s %s" % ( toolset[ 'size' ], outname ) )
  print "Generating binary image..."
  os.system( "%s -O binary %s %s.bin" % ( toolset[ 'bin' ], outname, output ) )

tools[ 'lm4f' ][ 'progfunc' ] = progfunc_lm4f
