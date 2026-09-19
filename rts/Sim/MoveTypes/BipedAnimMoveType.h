/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef BIPEDANIMMOVETYPE_H
#define BIPEDANIMMOVETYPE_H

#include <vector>
#include <unordered_map>

#include "GroundMoveType.h"

// Animation-driven bipedal locomotion.
//
// CGroundMoveType (the base class) stays the source of truth for goal/path
// selection, waypoint following, turning intent, and collision legality.
// This class replaces only the *speed integration* step: instead of turning
// wantedSpeed/deltaSpeed into a velocity vector and integrating position
// from it, the realized displacement each frame comes from the owner's own
// script/animation state (root-motion style), read back through the stance
// "anchor" pieces.
//
// Anchor pieces are declared once (whenever the gait state machine changes
// which pieces can act as stance anchors), via Spring.SetUnitAnchorPieces(),
// not re-queried through a script callin every frame. Which of the declared
// candidates are *active* this frame is derived from ordinary piece
// visibility (CUnitScript::IsPieceVisible(), i.e. the same hide/show state
// scripts already toggle for IK/stance purposes) -- an invisible anchor
// piece is simply ignored. All active anchors are weighted equally.
//
// IMPORTANT frame-order fact (see Game.cpp): eventHandler.GameFrame() --
// which is how Lua unit scripts get to move/hide/show pieces -- runs
// *before* CUnitHandler::UpdateUnitMoveTypes() in the same sim frame. This
// means the anchor visibility/deltas read this frame were produced by the
// script in reaction to the constraints computed *last* frame, not this
// one; there is an inherent 1-frame latency between "movetype asks" and
// "script answers". The system below is ordered to match that reality:
//
//   1. UpdateTraversalPlan()  - inherited unchanged; computes waypointDir /
//                               wantedHeading / wantedSpeed intent for path
//                               following (unrelated to anchors).
//   2. UpdateAnchorMotion()   - (re)computes wantedSpeed via ChangeSpeed(),
//                               then filters the declared anchor candidates
//                               by piece visibility (as the script already
//                               set it this frame) and turns the active
//                               set's motion into a world-space
//                               displacement request.
//   3. UpdateAnimUnitPosition() - run that request through UpdatePos() so
//                               terrain/static-object legality is enforced
//                               exactly like CGroundMoveType does.
//   4. UpdatePreCollisions() / UpdateCollisionDetections() - inherited
//                               unchanged (commit + unit-unit push forces).
//   5. Update()               - commit collision-pushed position and feed
//                               the realized motion back through
//                               UpdateOwnerSpeed() (StartMoving/StopMoving,
//                               push-resistance bookkeeping).
//
// This ordering is deliberate and load-bearing: per the design notes this
// class was built from, a fuzzy movetype<->script frame contract shows up
// as jitter / phase-lag oscillation, so don't reorder steps 2-5 without
// re-checking Game.cpp's Sim update order.
//
// Speed itself is produced entirely by the script's animation (root motion),
// never by an engine-side accel/decel ramp: accRate/decRate are forced to
// NO_ACCEL_DECEL_LIMIT (see ctor) so ChangeSpeed()'s deltaSpeed output -
// which this class never reads - can't gate anything even incidentally.
// Coming to a stop is expected to be gated by the gait finishing its current
// step, not by a decel curve. The script's only speed signal from the engine
// is the existing get/set CURRENT_SPEED unit-value (GetWantedSpeedFraction(),
// a 0-100% of max-speed target the gait should average, not an instantaneous
// speed -- Spring.GetUnitVelocity() remains instantaneous for everyone else).
class CBipedAnimMoveType : public CGroundMoveType
{
	CR_DECLARE_DERIVED(CBipedAnimMoveType)

public:
	CBipedAnimMoveType(CUnit* owner);

	bool Update() override;
	void Connect() override;
	void Disconnect() override;

	// UpdatePreCollisions() (inherited unchanged) would otherwise call this
	// and stomp currentSpeed with always-zero oldSpeed/newSpeed (this class
	// never runs UpdateUnitPosition(), the only thing that sets them); the
	// real UpdateOwnerSpeed() call for this class lives in Update() instead.
	void CommitOwnerSpeed() override {}

	// 0-100% of max speed the gait should average this frame; backs the
	// CURRENT_SPEED unit-value override in CUnitScript::GetUnitVal().
	float GetWantedSpeedFraction() const;

	// Declares which pieces (by script piece index) may act as stance
	// anchors; called rarely (only when the gait state machine's anchor set
	// changes), from Spring.SetUnitAnchorPieces. Which of these are *active*
	// on any given frame is derived from piece visibility, not from this list.
	void SetAnchorPieceCandidates(std::vector<int> pieces);

	// Step 2: filter the declared anchor candidates by piece visibility and
	// turn the active set's motion into a world-space displacement request.
	// 0 active anchors -> falls back to short inertia coasting from the last
	// realized velocity.
	void UpdateAnchorMotion();

	// Step 3: replaces CGroundMoveType::UpdateUnitPosition(). Runs the
	// pending displacement request through UpdatePos() (terrain/static
	// legality) and stages the legal result in resultantForces, exactly
	// like the base class does, so the inherited UpdatePreCollisions() /
	// UpdateCollisionDetections() commit it the same way.
	void UpdateAnimUnitPosition();

private:
	struct AnchorSample {
		float3 plantedWorldPos;   // world position where this anchor first became active (planted)
		bool planted = false;
	};

	// 0-anchor fallback: keep coasting along the last realized velocity,
	// decaying it multiplicatively (by INERTIA_DECAY) each frame.
	float3 SampleInertiaDelta() const;

private:
	std::vector<int> anchorPieceCandidates;   // declared via SetAnchorPieceCandidates(); most are inactive (invisible) at any given time
	std::unordered_map<int, AnchorSample> candidateStates;  // per-candidate tracking state, updated every frame regardless of activity
	float3 animMoveRequest;                   // pending world-space displacement, set by UpdateAnchorMotion()

	float3 lastRealizedVelocity;
	float3 inertiaVelocity;                   // decaying copy of lastRealizedVelocity, drives the 0-anchor fallback

	// safeguards (elmos/frame)
	static constexpr float MAX_DISPLACEMENT_PER_FRAME = 60.0f;
	static constexpr float INERTIA_DECAY = 0.98f;
	// large-but-finite rather than infinite: BrakingDistance() computes
	// rate*time*time with time=speed/rate, and inf*0 is NaN.
	static constexpr float NO_ACCEL_DECEL_LIMIT = 1e6f;
};

#endif // BIPEDANIMMOVETYPE_H
