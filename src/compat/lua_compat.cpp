extern "C" {
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

// Provide a minimal, weak compatibility implementation of luaL_openlibs so
// builds linking against different Lua exports succeed without editing
// third-party sources. This registers the common standard libraries.
#if defined(__GNUC__)
extern "C" __attribute__((weak)) void luaL_openlibs(lua_State *L) {
#else
extern "C" void luaL_openlibs(lua_State *L) {
#endif
    if (!L)
        return;

#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 502
    luaL_requiref(L, "base", luaopen_base, 1);
    lua_pop(L, 1);
    luaL_requiref(L, "package", luaopen_package, 1);
    lua_pop(L, 1);
#ifdef luaopen_coroutine
    luaL_requiref(L, "coroutine", luaopen_coroutine, 1);
    lua_pop(L, 1);
#endif
    luaL_requiref(L, "string", luaopen_string, 1);
    lua_pop(L, 1);
    luaL_requiref(L, "table", luaopen_table, 1);
    lua_pop(L, 1);
    luaL_requiref(L, "math", luaopen_math, 1);
    lua_pop(L, 1);
#ifdef luaopen_io
    luaL_requiref(L, "io", luaopen_io, 1);
    lua_pop(L, 1);
#endif
#ifdef luaopen_os
    luaL_requiref(L, "os", luaopen_os, 1);
    lua_pop(L, 1);
#endif
#ifdef luaopen_debug
    luaL_requiref(L, "debug", luaopen_debug, 1);
    lua_pop(L, 1);
#endif
#ifdef luaopen_utf8
    luaL_requiref(L, "utf8", luaopen_utf8, 1);
    lua_pop(L, 1);
#endif
#else
    (void)L;
#endif
}
