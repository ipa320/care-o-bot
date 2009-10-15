/// @file GlobalDefines.h
/// Basic global defines have to be put in this file within the namespace ipa.utils.
/// If they do not fit within this file, consider integrating your defines within a class.
/// This file has been written by Jan Fischer in 2008.
/// Latest updates: November 2008.

#ifndef __GLOBALDEFINES_H__
#define __GLOBALDEFINES_H__

namespace ipa_Utils {
	
/// An enum for the return values.
/// This enum describes possible return values that are used to return failure or success.
enum {
	RET_OK =									0x00000001UL, ///< Everythings OK.
	RET_FAILED =								0x00000002UL  ///< Something went wrong.
};

/// A definition for PI.
static const double THE_PI_DEF = 3.141593;
/// A definition for Epsilon (very small value).
static const double THE_EPS_DEF = 0.000001;


} // end namespace ipa_Utils

#endif // __GLOBALDEFINES_H__

