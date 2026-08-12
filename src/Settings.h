// Copyright © 2023 CCP ehf.

#pragma once
#ifndef DESTINY_SETTINGS_H
#define DESTINY_SETTINGS_H
#include "StdAfx.h"


constexpr size_t g_collisionMaxIterationsDefault = 20;
constexpr bool g_useIterativeCollisionDefault = false;
constexpr bool g_useDynamicalOrientationDefault = false;
constexpr bool g_disableDynamicalOrientationForMissilesDefault = false;
constexpr bool g_useNewOrbitDefault = false;

// Only do this many iterations in the search for collision before stopping.
extern size_t g_collisionMaxIterations;
extern bool g_useIterativeCollision;
extern bool g_useDynamicalOrientation;
extern bool g_disableDynamicalOrientationForMissiles;
extern bool g_useNewOrbit;


BLUE_CLASS( SettingsConfiguration ) : public IRoot
{
public:
	EXPOSE_TO_BLUE();
	size_t m_collisionMaxIterations = g_collisionMaxIterationsDefault;
	bool m_useIterativeCollision = g_useIterativeCollisionDefault;
	bool m_useDynamicalOrientation = g_useDynamicalOrientationDefault;
	bool m_disableDynamicalOrientationForMissiles = g_disableDynamicalOrientationForMissilesDefault;
	bool m_useNewOrbit = g_useNewOrbitDefault;
};
TYPEDEF_BLUECLASS( SettingsConfiguration );

BLUE_CLASS( GlobalSettings ) : public IRoot
{
public:
	EXPOSE_TO_BLUE();
	PyObject* ApplyConfiguration( PyObject * args );
	PyObject* GetConfiguration( PyObject * args );
	PyObject* GetDefaultConfiguration( PyObject* args );
	PyObject* ResetConfiguration( PyObject * args );
};

TYPEDEF_BLUECLASS( GlobalSettings );

extern GlobalSettings* s_globalSettings;

#endif
