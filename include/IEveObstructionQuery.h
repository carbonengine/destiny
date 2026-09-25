// Copyright © 2026 Fenris Creations ehf.

#pragma once

struct Vector3;

/**
 * @brief Interface for CarbonAudio's line of sight occlusion queries.
 *
 * CarbonAudio sends listener and emitter position data to destiny and gets back whether each
 * emitter is occluded.
 */
BLUE_INTERFACE( IEveObstructionQuery ) : public IRoot
{
	/**
	 * @brief Check line of sight from the source to each target.
	 *
	 *
	 * @param source      Listener position.
	 * @param targets     Emitter positions, targetCount of them.
	 * @param targetCount Number of entries in targets and in outBlocked.
	 * @param outBlocked  One entry per target, set to true when the sightline is blocked.
	 *
	 * @return True if outBlocked was written.
	 */
	virtual bool QuerySightlines( const Vector3& source, const Vector3* targets, unsigned int targetCount, bool* outBlocked ) = 0;
};
