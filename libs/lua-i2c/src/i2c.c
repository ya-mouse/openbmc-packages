/*
License: CC BY-SA (http://creativecommons.org/licenses/by-sa/2.5/)
This license lets others remix, tweak, and build upon a work even for commercial reasons, as long as they credit the original author and license their new creations under the identical terms.

Orginal Author: Mikael Sundin, se.linkedin.com/in/mikaelsundin
*/

#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"

/* return values from I2Clib_rw */
enum{
    RW_ACK = 0,
    RW_NACK = 1,
    RW_ERROR_SEND = 2,
    RW_ERROR_BUS = 3,
    RW_ERROR_PARAM = 4
};

/* prototypes */
static int libluai2c_rw( int bus, int addr, const char *wptr, int wlen, char *rptr, int rlen);
LUALIB_API int libluai2c_write(lua_State *L);
LUALIB_API int libluai2c_read(lua_State *L);
LUALIB_API int libluai2c_version(lua_State *L);


LUALIB_API int libluai2c_version(lua_State *L){
    lua_pushstring(L, "libluai2c version 0.9, Mikael Sundin 2013");
    
    return 1;
}

LUALIB_API int libluai2c_help(lua_State *L){
    lua_pushstring(L, "usage:\n"
                      "status = write(bus(int), device(int), data(string))\n"
                      "status, data = read(bus(int), device(int), read length(int), [data(string)])\n"
                      "   (optional data is written before read)\n"
                      "   status ok: ack=0\n"
                      "   status error: nack=1, send error=2, bus error=3, parameter error=4\n"
                      "version - version string\n");
    return 1;
}


/* write and/or read i2c bus 
* @param bus, I2c bus to work on
* @param addr, i2c device address
* @param wptr, pointer to data to write to
* @param wlen, write length
* @param rptr, pointer to store read data to
* @param rlen, number of bytes to read
* @return: 0=ack, 1=nack, 2=bus error, 3=parameter error
*/
int libluai2c_rw(  int bus,
                int addr, 
                const char *wptr, int wlen, 
                char *rptr, int rlen){
    int f;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    char dev[15];

    if(wlen == 0 && rlen == 0){
        return RW_ERROR_PARAM;
    }
    
    sprintf(&dev[0], "/dev/i2c-%d", bus);
    
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = wlen;
    messages[0].buf   = (char *)wptr;
    
    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = rlen;
    messages[1].buf   = rptr;
    
    if(wlen > 0 && rlen > 0){
        packets.msgs = &messages[0];    //write & read
        packets.nmsgs = 2;
    }else if(wlen > 0){         
        packets.msgs = &messages[0];    //only write    
        packets.nmsgs = 1;
    }else if(rlen > 0){         
        packets.msgs = &messages[1];    //only read
        packets.nmsgs = 1;
    }
   
    if ((f = open(dev, O_RDWR)) < 0) {
        return RW_ERROR_BUS;
    }

    if(ioctl(f, I2C_RDWR, &packets) < 0) {
        close(f);
        return RW_ERROR_SEND;
    }

    close(f);
    
    /* check for error from i2c transfer */
    if( (messages[0].flags & I2C_M_NO_RD_ACK) || 
        (messages[1].flags & I2C_M_NO_RD_ACK)){
        return RW_NACK;    
    }else{
        return RW_ACK;
    }
}

/* LUA parameters:
 * int i2c bus number
 * int address
 * string data to write
 *
 * Return values:
 * status
 * Number of bytes written
 */
LUALIB_API int libluai2c_write(lua_State *L){
    int args;
    int bus;
    int address;
    
    int wlen;
    const char *wptr;

    if( lua_gettop(L) < 3){
        return luaL_error(L, "Wrong number of arguments");
    }
    bus = lua_tointeger (L, 1);
    address = lua_tointeger(L, 2);
    wptr = lua_tolstring(L, 3, &wlen);
    
    //write back i2c write status
    lua_pushnumber(L, libluai2c_rw(bus, address, wptr, wlen, 0, 0));
    return 1;
}

/* LUA 
 * Parameters:
 * int bus
 * int address
 * int length
 * optional string data to write before read (probably set i2c device internal address or so)
 *
 * Return:
 * Status
 * int number of bytes read
 * string data read from device
 */
LUALIB_API int libluai2c_read(lua_State *L){
    int bus;
    int address;
    int rlen;
    int wlen=0;
    const char *wptr=0;

    if( lua_gettop(L) < 3 ){
        return luaL_error(L, "Wrong number of arguments");
    }
    
    //parse input
    bus = lua_tointeger (L, 1);
    address = lua_tointeger(L, 2);
    rlen = lua_tointeger(L, 3);
    wptr = (lua_gettop(L) == 4) ? lua_tolstring(L, 4, &wlen) : 0;
    
    /* transfer data */
    {
        char rdata[rlen];
        
        lua_pushnumber(L, libluai2c_rw(bus, address, wptr, wlen, &rdata[0], rlen));
        lua_pushlstring(L, &rdata[0], rlen);
        return 2;
    }
    
}

/* functions exposed to lua */
static const luaL_reg libluai2c_functions[] = {
  {"write", libluai2c_write},
  {"read", libluai2c_read},
  {"version", libluai2c_version},
  {"help", libluai2c_help},
  {NULL, NULL}
};


/* init function, will be called when lua run require */
LUALIB_API int luaopen_i2c (lua_State *L) {
    luaL_openlib(L, "i2c", libluai2c_functions, 0);
    return 1; 
}
