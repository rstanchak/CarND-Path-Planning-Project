#ifndef UWS_COMPAT_H
#define UWS_COMPAT_H

#include <uWS/uWS.h>

// silly wrappers to make callbacks in UWS 0.13 format work in 0.14 too
#ifdef CONFIG_UWS13_COMPAT
std::function<void(uWS::WebSocket<uWS::SERVER> *, char *, size_t, uWS::OpCode)> UWS_ON_MESSAGE( std::function<void(uWS::WebSocket<uWS::SERVER>, char *, size_t, uWS::OpCode)> handler ) 
{
    return [handler](uWS::WebSocket<uWS::SERVER> * ws, char *message, size_t length, uWS::OpCode opCode) {
       return handler(*ws, message, length, opCode);
    };
}

std::function<void(uWS::WebSocket<uWS::SERVER> *, uWS::HttpRequest)>
UWS_ON_CONNECTION( std::function<void(uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest)> handler ) 
{
    return [handler](uWS::WebSocket<uWS::SERVER> * ws, uWS::HttpRequest request) {
       return handler(*ws, request);
    };
}

std::function<void(uWS::WebSocket<uWS::SERVER> *, int code, char * message, size_t length)>
UWS_ON_DISCONNECTION( std::function<void(uWS::WebSocket<uWS::SERVER>, int code, char *, size_t)> handler ) 
{
    return [handler](uWS::WebSocket<uWS::SERVER> * ws, int code, char * message, size_t length) {
       return handler(*ws, code, message, length );
    };
}
#else
std::function<void(uWS::WebSocket<uWS::SERVER>, char *, size_t, uWS::OpCode)> UWS_ON_MESSAGE( std::function<void(uWS::WebSocket<uWS::SERVER>, char *, size_t, uWS::OpCode)> handler ) 
{
    return handler;
}

std::function<void(uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest)>
UWS_ON_CONNECTION( std::function<void(uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest)> handler ) 
{
    return handler;
}

std::function<void(uWS::WebSocket<uWS::SERVER>, int code, char * message, size_t length)>
UWS_ON_DISCONNECTION( std::function<void(uWS::WebSocket<uWS::SERVER>, int code, char *, size_t)> handler ) 
{
    return handler;
}
#endif

#endif
