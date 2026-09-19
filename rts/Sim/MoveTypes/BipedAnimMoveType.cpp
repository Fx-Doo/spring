/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "BipedAnimMoveType.h"

#include "Components/MoveTypesComponents.h"
#include "Rendering/Models/3DModelPiece.hpp"
#include "Sim/Ecs/Registry.h"
#include "Sim/Misc/GlobalSynced.h"
#include "Sim/Units/Scripts/UnitScript.h"
#include "Sim/Units/Unit.h"
#include "System/FastMath.h"
#include "System/SpringMath.h"
#include "System/Threading/ThreadPool.h"
#include "System/Misc/TracyDefs.h"
#include "System/Log/ILog.h"

using namespace MoveTypes;

CR_BIND_DERIVED(CBipedAnimMoveType, CGroundMoveType, (nullptr))
CR_REG_METADATA(CBipedAnimMoveType, (
	// transient per-frame state, fully recomputed from piece visibility on
	// the next Update(); nothing here needs to survive a save/load
	// round-trip. anchorPieceCandidates is simplified the same way for now --
	// a real save/load path would want to persist it.
	CR_IGNORED(anchorPieceCandidates),
	CR_IGNORED(candidateStates),
	CR_IGNORED(animMoveRequest),
	CR_IGNORED(lastRealizedVelocity),
	CR_IGNORED(inertiaVelocity)
))


CBipedAnimMoveType::CBipedAnimMoveType(CUnit* owner)
	: CGroundMoveType(owner)
{
	LOG_L(L_WARNING, "[%s] BipedAnimMoveType.cpp loaded, constructing for unit %i", __func__, owner->id);

	// speed is produced by the gait animation alone; make sure nothing (not
	// even ChangeSpeed()'s unused deltaSpeed output) can impose a kinematic
	// accel/decel ramp on top of it.
	accRate = NO_ACCEL_DECEL_LIMIT;
	decRate = NO_ACCEL_DECEL_LIMIT;
}


void CBipedAnimMoveType::Connect()
{
	RECOIL_DETAILED_TRACY_ZONE;
	LOG_L(L_WARNING, "[%s] BipedAnimMoveType running, connecting unit %i", __func__, owner->id);

	// Register under our own ECS tag, *not* MoveTypes::GroundMoveType, so
	// GroundMoveSystem's view doesn't also try to process us through the
	// base class's (non-virtual) Update*() methods.
	Sim::registry.emplace_or_replace<BipedAnimMoveType>(owner->entityReference, owner->id);
	Sim::registry.emplace_or_replace<FeatureCollisionEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<UnitCollisionEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<FeatureCrushEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<UnitCrushEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<FeatureMoveEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<UnitMovedEvent>(owner->entityReference);
	Sim::registry.emplace_or_replace<ChangeHeadingEvent>(owner->entityReference, owner->id);
	Sim::registry.emplace_or_replace<ChangeMainHeadingEvent>(owner->entityReference, owner->id);
}

void CBipedAnimMoveType::Disconnect()
{
	RECOIL_DETAILED_TRACY_ZONE;
	Sim::registry.remove<BipedAnimMoveType>(owner->entityReference);
	Sim::registry.remove<FeatureCollisionEvents>(owner->entityReference);
	Sim::registry.remove<UnitCollisionEvents>(owner->entityReference);
	Sim::registry.remove<FeatureCrushEvents>(owner->entityReference);
	Sim::registry.remove<UnitCrushEvents>(owner->entityReference);
	Sim::registry.remove<FeatureMoveEvents>(owner->entityReference);
	Sim::registry.remove<UnitMovedEvent>(owner->entityReference);
	Sim::registry.remove<ChangeHeadingEvent>(owner->entityReference);
	Sim::registry.remove<ChangeMainHeadingEvent>(owner->entityReference);
}


void CBipedAnimMoveType::SetAnchorPieceCandidates(std::vector<int> pieces)
{
	RECOIL_DETAILED_TRACY_ZONE;
	anchorPieceCandidates.clear();
	anchorPieceCandidates.reserve(pieces.size());

	for (const int lmodelPiece : pieces) {
		const bool inRange = (lmodelPiece >= 0 && lmodelPiece < (int)owner->localModel.pieces.size());
		// candidates arrive as LocalModel piece indices (Lua/model numbering),
		// but IsPieceVisible()/GetPiecePos() below key off script piece
		// indices (COB declaration-order numbering) -- these two numbering
		// spaces are unrelated, so every candidate must be converted once.
		const int scriptPiece = owner->script->ModelToScript(lmodelPiece);

		LOG_L(L_WARNING, "[%s] unit %i registers anchor candidate lmodel piece %i (name=%s) -> script piece %i", __func__, owner->id, lmodelPiece,
			inRange ? owner->localModel.pieces[lmodelPiece].original->name.c_str() : "OUT-OF-RANGE", scriptPiece);

		if (scriptPiece >= 0)
			anchorPieceCandidates.push_back(scriptPiece);
	}
	candidateStates.clear();
}


float CBipedAnimMoveType::GetWantedSpeedFraction() const
{
	const float maxSpeed = GetMaxSpeed();
	return (maxSpeed > 0.0f) ? std::clamp(wantedSpeed / maxSpeed, 0.0f, 1.0f) : 0.0f;
}


void CBipedAnimMoveType::UpdateAnchorMotion()
{
	RECOIL_DETAILED_TRACY_ZONE;
	LOG_L(L_WARNING, "[%s] entered for unit %i", __func__, owner->id);

	// Only used for its wantedSpeed side effect (terrain/turn/braking
	// modifiers); deltaSpeed is never read since accRate/decRate are no-ops.
	switch (setHeading) {
		case HEADING_CHANGED_MOVE:
			ChangeSpeed(maxWantedSpeed, WantReverse(waypointDir, flatFrontDir));
			setHeading = HEADING_CHANGED_NONE;
			break;
		case HEADING_CHANGED_STOP:
		case HEADING_CHANGED_STUN:
			ChangeSpeed(0.0f, false);
			setHeading = HEADING_CHANGED_NONE;
			break;
	}

	// Track every declared candidate continuously (active or not). When one
	// becomes active (planted), its current world position is captured as
	// the lock target; while it stays active, the unit is moved by however
	// far the anchor has drifted from that locked point, so it stays pinned
	// to the ground contact point rather than free-sliding with the model.
	float3 summedWorldDelta;
	int activeCount = 0;

	for (const int piece : anchorPieceCandidates) {
		AnchorSample& st = candidateStates[piece];

		const bool active = owner->script->IsPieceVisible(piece);
		LOG_L(L_WARNING, "[%s][f=%d] unit %i anchor candidate piece %i visibility=%d", __func__, gs->frameNum, owner->id, piece, (int)active);

		const float3 curWorldPos = owner->GetObjectSpacePos(owner->script->GetPiecePos(piece));

		if (active) {
			if (!st.planted) {
				st.plantedWorldPos = curWorldPos;
				st.planted = true;
			} else {
				// How far the anchor has drifted from where it was planted;
				// move the unit to cancel that drift out.
				summedWorldDelta += (st.plantedWorldPos - curWorldPos);
				activeCount++;
			}
		} else {
			st.planted = false;
		}
	}

	float3 worldDelta;
	if (activeCount > 0) {
		worldDelta = summedWorldDelta / float(activeCount);
		inertiaVelocity = lastRealizedVelocity;
	} else {
		worldDelta = SampleInertiaDelta();
	}

	if (worldDelta.SqLength() > Square(MAX_DISPLACEMENT_PER_FRAME))
		worldDelta = worldDelta.Normalize() * MAX_DISPLACEMENT_PER_FRAME;

	animMoveRequest = worldDelta;
	LOG_L(L_WARNING, "[%s][f=%d] unit %i candidates=%i active=%i animMoveRequest=(%.3f,%.3f,%.3f)", __func__, gs->frameNum, owner->id, (int)anchorPieceCandidates.size(), activeCount, animMoveRequest.x, animMoveRequest.y, animMoveRequest.z);
}


float3 CBipedAnimMoveType::SampleInertiaDelta() const
{
	RECOIL_DETAILED_TRACY_ZONE;
	return inertiaVelocity;
}


void CBipedAnimMoveType::UpdateAnimUnitPosition()
{
	RECOIL_DETAILED_TRACY_ZONE;
	resultantForces = ZeroVector;

	if (owner->IsSkidding() || owner->IsFalling())
		return;
	if (owner->GetTransporter() != nullptr)
		return;
	if (animMoveRequest.same(ZeroVector))
		return;

	float3 resultantMove;
	UpdatePos(owner, animMoveRequest, resultantMove, ThreadPool::GetThreadNum());

	// resultantMove may differ from animMoveRequest (terrain/static-object
	// legality); the collision-corrected value is authoritative, per design.
	resultantForces = resultantMove;
}


bool CBipedAnimMoveType::Update()
{
	RECOIL_DETAILED_TRACY_ZONE;
	LOG_L(L_WARNING, "[%s] BipedAnimMoveType::Update running for unit %i", __func__, owner->id);

	if (owner->requestRemoveUnloadTransportId) {
		owner->unloadingTransportId = -1;
		owner->requestRemoveUnloadTransportId = false;
	}

	SyncWaypoints();

	if (owner->GetTransporter() != nullptr) return false;
	if (owner->IsSkidding()) return false;
	if (owner->IsFalling()) return false;

	// resultantForces here holds the unit-unit/feature collision push
	// computed by the inherited UpdateCollisionDetections(); the anchor
	// displacement itself was already committed by the inherited
	// UpdatePreCollisions() earlier this frame.
	if (resultantForces.SqLength() > 0.f)
		owner->Move(resultantForces, true);

	AdjustPosToWaterLine();

	const float3 realizedDelta = owner->pos - oldPos;
	const float signedSpeed = realizedDelta.dot(flatFrontDir);
	const float realizedSpeed = math::fabs(signedSpeed);

	// Root-motion, not integrated velocity: feed the realized motion back
	// so script StartMoving()/StopMoving() and push-resistance bookkeeping
	// stay correct, then expose it as the owner's velocity for everyone
	// else (weapon lead, camera, etc).
	reversing = UpdateOwnerSpeed(math::fabs(currentSpeed), realizedSpeed, signedSpeed);
	owner->SetVelocityAndSpeed(realizedDelta);
	currentSpeed = realizedSpeed;

	lastRealizedVelocity = realizedDelta;
	inertiaVelocity *= INERTIA_DECAY;

	return OwnerMoved(owner->heading, realizedDelta, float3(float3::cmp_eps(), float3::cmp_eps() * 1e-2f, float3::cmp_eps()));
}
