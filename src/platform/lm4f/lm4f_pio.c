// LM4F specific PIO support

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "inc/hw_gpio.h"
#include "pin_map.h"
#include "gpio.h"
#include "pin_map.h"
#include "auxmods.h"
#include <string.h>

/* TODO: I'll leave this restriction (LUA_OPTIMIZE_MEMORY != 0) 
 *      for safety reason. It must be analysed again.
 */
#if LUA_OPTIMIZE_MEMORY == 0
#error lm4f.pio can only be compiled with LTR on (optram=true)
#endif

// Alternate function setting is available in LM4F120H5QR
// Reference: see document spms294e, pg. 609.
#define LM4F_HAS_ALTERNATE_PIO

// This define is necessary to use the GPIO defines of LM4F120H5QR.
// Reference: see file pin_map.h, line 49818
#define PART_LM4F120H5QR

// ****************************************************************************
// Alternate function handling
#ifdef LM4F_HAS_ALTERNATE_PIO

// List all alternative pin functions here.
// Reference: see file pin_map.h, line 49818
#define LM4F_ALTERNATE_FUNCTIONS\
  _M( PA0_U0RX ),\
  _M( PA1_U0TX ),\
  _M( PA2_SSI0CLK ),\
  _M( PA3_SSI0FSS ),\
  _M( PA4_SSI0RX ),\
  _M( PA5_SSI0TX ),\
  _M( PA6_I2C1SCL ),\
  _M( PA7_I2C1SDA ),\
  _M( PB0_U1RX ),\
  _M( PB0_T2CCP0 ),\
  _M( PB1_U1TX ),\
  _M( PB1_T2CCP1 ),\
  _M( PB2_I2C0SCL ),\
  _M( PB2_T3CCP0 ),\
  _M( PB3_I2C0SDA ),\
  _M( PB3_T3CCP1 ),\
  _M( PB4_SSI2CLK ),\
  _M( PB4_CAN0RX ),\
  _M( PB4_T1CCP0 ),\
  _M( PB5_SSI2FSS ),\
  _M( PB5_CAN0TX ),\
  _M( PB5_T1CCP1 ),\
  _M( PB6_SSI2RX ),\
  _M( PB6_T0CCP0 ),\
  _M( PB7_SSI2TX ),\
  _M( PB7_T0CCP1 ),\
  _M( PC0_TCK ),\
  _M( PC0_SWCLK ),\
  _M( PC0_T4CCP0 ),\
  _M( PC1_TMS ),\
  _M( PC1_SWDIO ),\
  _M( PC1_T4CCP1 ),\
  _M( PC2_TDI ),\
  _M( PC2_T5CCP0 ),\
  _M( PC3_SWO ),\
  _M( PC3_TDO ),\
  _M( PC3_T5CCP1 ),\
  _M( PC4_U4RX ),\
  _M( PC4_U1RX ),\
  _M( PC4_WT0CCP0 ),\
  _M( PC4_U1RTS ),\
  _M( PC5_U4TX ),\
  _M( PC5_U1TX ),\
  _M( PC5_WT0CCP1 ),\
  _M( PC5_U1CTS ),\
  _M( PC6_U3RX ),\
  _M( PC6_WT1CCP0 ),\
  _M( PC7_U3TX ),\
  _M( PC7_WT1CCP1 ),\
  _M( PD0_SSI3CLK ),\
  _M( PD0_SSI1CLK ),\
  _M( PD0_I2C3SCL ),\
  _M( PD0_WT2CCP0 ),\
  _M( PD1_SSI3FSS ),\
  _M( PD1_SSI1FSS ),\
  _M( PD1_I2C3SDA ),\
  _M( PD1_WT2CCP1 ),\
  _M( PD2_SSI3RX ),\
  _M( PD2_SSI1RX ),\
  _M( PD2_WT3CCP0 ),\
  _M( PD3_SSI3TX ),\
  _M( PD3_SSI1TX ),\
  _M( PD3_WT3CCP1 ),\
  _M( PD4_U6RX ),\
  _M( PD4_WT4CCP0 ),\
  _M( PD5_U6TX ),\
  _M( PD5_WT4CCP1 ),\
  _M( PD6_U2RX ),\
  _M( PD6_WT5CCP0 ),\
  _M( PD7_U2TX ),\
  _M( PD7_WT5CCP1 ),\
  _M( PD7_NMI ),\
  _M( PE0_U7RX ),\
  _M( PE1_U7TX ),\
  _M( PE4_U5RX ),\
  _M( PE4_I2C2SCL ),\
  _M( PE4_CAN0RX ),\
  _M( PE5_U5TX ),\
  _M( PE5_I2C2SDA ),\
  _M( PE5_CAN0TX ),\
  _M( PF0_U1RTS ),\
  _M( PF0_SSI1RX ),\
  _M( PF0_CAN0RX ),\
  _M( PF0_T0CCP0 ),\
  _M( PF0_NMI ),\
  _M( PF0_C0O ),\
  _M( PF0_TRD2 ),\
  _M( PF1_U1CTS ),\
  _M( PF1_SSI1TX ),\
  _M( PF1_T0CCP1 ),\
  _M( PF1_C1O ),\
  _M( PF1_TRD1 ),\
  _M( PF2_T1CCP0 ),\
  _M( PF2_SSI1CLK ),\
  _M( PF2_TRD0 ),\
  _M( PF3_CAN0TX ),\
  _M( PF3_T1CCP1 ),\
  _M( PF3_SSI1FSS ),\
  _M( PF3_TRCLK ),\
  _M( PF4_T2CCP0 ),

typedef struct
{
  const char *name;
  u32 val;
} LM4F_PIN_DATA;

	#define _M( x )   { #x, #x }
static const LM4F_PIN_DATA lm4f_pin_data[] = 
{
  LM4F_ALTERNATE_FUNCTIONS
  { NULL, 0 }
};

static int lm4f_pio_mt_index( lua_State *L )
{
  const char *key = luaL_checkstring( L, 2 );
  unsigned i = 0;
  
  while( lm4f_pin_data[ i ].name != NULL )
  {
    if( !strcmp( lm4f_pin_data[ i ].name, key ) )
    {
      lua_pushnumber( L, ( lua_Number )lm4f_pin_data[ i ].val );
      return 1;
    }
    i ++;
  }
  return 0;
}

// Lua: lm4f.pio.set_function( func1, func2, ..., funcn )
static int lm4f_pio_set_function( lua_State *L )
{
  unsigned i;

  for( i = 1; i <= lua_gettop( L ); i ++ )
    GPIOPinConfigure( ( u32 )luaL_checknumber( L, i ) );
  return 0;
}

#endif // #ifdef LM4F_HAS_ALTERNATE_PIO

// ****************************************************************************
// Other LM4F PIO specific functions

extern const u32 pio_base[];

// Helper: check a port/pin specification
// Return 1 if OK, 0 if false
// Set port and pin in args as side effect
static int lm4f_pioh_check_pio_spec( int v, int *pport, int *ppin )
{
  *pport = PLATFORM_IO_GET_PORT( v );
  *ppin = PLATFORM_IO_GET_PIN( v );
  if( PLATFORM_IO_IS_PORT( v ) || !platform_pio_has_port( *pport ) || !platform_pio_has_pin( *pport, *ppin ) )
    return 0;
  return 1;
}

// Lua: lm4f.pio.set_strength( drive, pin1, pin2, ..., pinn )
static int lm4f_pio_set_strength( lua_State *L )
{
  int port = 0, pin = 0;
  u8 pins;
  u32 base;
  u32 drive = luaL_checkinteger( L, 1 );
  unsigned i;
  
  for( i = 2; i <= lua_gettop( L ); i ++ )
  {
    if( !lm4f_pioh_check_pio_spec( luaL_checkinteger( L, i ), &port, &pin ) )
      return luaL_error( L, "invalid pin '%u'", luaL_checkinteger( L, i ) );
    base = pio_base[ port ];
    pins = 1 << pin;
    // The next sequence is taken from gpio.c
	// Reference: gpio.c, line 531.
    HWREG(base + GPIO_O_DR2R) = ((drive & 1) ?
                                   (HWREG(base + GPIO_O_DR2R) | pins) :
                                   (HWREG(base + GPIO_O_DR2R) & ~(pins)));
    HWREG(base + GPIO_O_DR4R) = ((drive & 2) ?
                                   (HWREG(base + GPIO_O_DR4R) | pins) :
                                   (HWREG(base + GPIO_O_DR4R) & ~(pins)));
    HWREG(base + GPIO_O_DR8R) = ((drive & 4) ?
                                   (HWREG(base + GPIO_O_DR8R) | pins) :
                                   (HWREG(base + GPIO_O_DR8R) & ~(pins)));
    HWREG(base + GPIO_O_SLR) = ((drive & 8) ?
                                  (HWREG(base + GPIO_O_SLR) | pins) :
                                  (HWREG(base + GPIO_O_SLR) & ~(pins)));
  }
  return 0;
}

// Lua: lm4f.pio.set_direction( dir, pin1, pin2, ..., pinn )
static int lm4f_pio_set_direction( lua_State *L )
{
  int port = 0, pin = 0;
  u32 base, dir;
  u8 pins;
  unsigned i;

  dir = ( u32 )luaL_checkinteger( L, 1 );
  for( i = 2; i <= lua_gettop( L ); i ++ )
  {
    if( !lm4f_pioh_check_pio_spec( luaL_checkinteger( L, i ), &port, &pin ) )
      return luaL_error( L, "invalid pin '%u'", luaL_checkinteger( L, i ) );
    base = pio_base[ port ];
    pins = 1 << pin;
    GPIODirModeSet( base, pins, dir );
    HWREG( base  + GPIO_O_DEN ) |= pins;
  }
  return 0;
}

// Module function map
/* TODO: analyse this MIN_OPT_LEVEL */
#define MIN_OPT_LEVEL 2
#include "lrodefs.h" 
const LUA_REG_TYPE lm4f_pio_map[] =
{
#ifdef LM4F_HAS_ALTERNATE_PIO
  { LSTRKEY( "__index" ), LFUNCVAL( lm4f_pio_mt_index ) },
  { LSTRKEY( "__metatable" ), LROVAL( lm4f_pio_map ) },
  { LSTRKEY( "set_function" ), LFUNCVAL( lm4f_pio_set_function ) },
#endif // #ifdef LM4F_HAS_ALTERNATE_PIO
  { LSTRKEY( "set_strength" ),  LFUNCVAL( lm4f_pio_set_strength ) },
  { LSTRKEY( "MA_2" ), LNUMVAL( GPIO_STRENGTH_2MA ) },
  { LSTRKEY( "MA_4" ), LNUMVAL( GPIO_STRENGTH_4MA ) },
  { LSTRKEY( "MA_8" ), LNUMVAL( GPIO_STRENGTH_8MA ) },
  { LSTRKEY( "MA_8SC" ), LNUMVAL( GPIO_STRENGTH_8MA_SC ) },
  { LSTRKEY( "set_direction" ), LFUNCVAL( lm4f_pio_set_direction ) },
  { LSTRKEY( "GPIO_IN" ), LNUMVAL( GPIO_DIR_MODE_IN ) },
  { LSTRKEY( "GPIO_OUT" ), LNUMVAL( GPIO_DIR_MODE_OUT ) },
  { LSTRKEY( "HW" ), LNUMVAL( GPIO_DIR_MODE_HW ) },
  { LNILKEY, LNILVAL }
};

