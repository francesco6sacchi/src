#pragma once

#ifndef __H_INCLUDE_EXCEPTIONS__
#define __H_INCLUDE_EXCEPTIONS__

#define SMARTKIT_ALREADY_INSTANTIATED_EXCEPTION 0

#define XML_FILE_EXCEPTION                      1
#define XML_PARSING_EXCEPTION                   2
#define XML_PARAMETER_EXCEPTION                 3

#define INTERFACE_INIT_EXCEPTION                4
#define INTERFACE_RECV_EXCEPTION                5
#define INTERFACE_SEND_EXCEPTION                6

#define SENSOR_INIT_EXCEPTION                   7
#define SENSOR_VOICE_EXCEPTION                  8
#define SENSOR_DEPTH_EXCEPTION                  9
#define SENSOR_BODY_EXCEPTION                   10
#define SENSOR_INFRARED_EXCEPTION               11
#define SENSOR_RGB_EXCEPTION                    12
#define SENSOR_NOT_DEFINED                      13

#define COLLINEAR_POINTS_EXCEPTION              14
#define EMPTY_POINT_CLOUD_EXCEPTION             15
#define POINT_CLOUD_NAN                         16
#define CHULL_EXCEPTION                         17
#define NON_MACHING_DIMENSIONS_EXECPTION        18

#define FILE_HANDLING_EXCEPTION                 19

#define EMPTY_STRING_EXCEPTION                  20
#define UNKNOWN_OPERATOR_EXCEPTION              21
#define BAD_PARENTHESIZATION_EXCEPTION          22

#define INCONSISTENT_SIGNAL_VALIDITY            23
#define UNDEFINED_SIGNAL_EXCEPTION              24
#define BAD_PROPOSITION_EXCEPTION               25

#define OUT_OF_RANGE_PARAMETER                  26
#define NOT_IMPLEMENTED_METHOD_EXCEPTION        27
#define NON_NEGATIVE_VALUE_EXCEPTION            28

#define DAQ_EXCEPTION							29

#define NULL_POINTER_EXCEPTION					99



#endif
