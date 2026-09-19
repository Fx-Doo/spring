/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "BipedAnimMoveSystem.h"

#include "Sim/Ecs/Registry.h"
#include "Sim/Features/Feature.h"
#include "Sim/Misc/GlobalSynced.h"
#include "Sim/Misc/QuadField.h"
#include "Sim/MoveTypes/BipedAnimMoveType.h"
#include "Sim/MoveTypes/Components/MoveTypesComponents.h"
#include "Sim/Units/Unit.h"
#include "Sim/Units/UnitHandler.h"

#include "System/EventHandler.h"
#include "System/TimeProfiler.h"
#include "System/Threading/ThreadPool.h"
#include "System/Log/ILog.h"

using namespace MoveTypes;

void BipedAnimMoveSystem::Init() {}

template<typename T, typename F>
static void issue_events(F func)
{
    auto view = Sim::registry.view<T>();
    view.each([&](T& comp){
        std::for_each(comp.value.begin(), comp.value.end(), func);
        comp.value.clear();
    });
}

// Structurally mirrors GroundMoveSystem::Update()'s phase layout; see
// BipedAnimMoveType.h for why anchor motion is sampled in phase 1 (right
// alongside path/turn planning) rather than assumed same-frame with the
// script -- piece visibility this frame reflects the script's reaction to
// whatever CURRENT_SPEED it queried during the *previous* sim frame's
// eventHandler.GameFrame() call.
void BipedAnimMoveSystem::Update() {
    {
        SCOPED_TIMER("Sim::Unit::MoveType::Biped::1::UpdateTraversalPlan");
        auto view = Sim::registry.view<BipedAnimMoveType>();
        for_mt(0, view.size(), [&view](const int i){
            auto entity = view.storage<BipedAnimMoveType>()[i];
            auto unitId = view.get<BipedAnimMoveType>(entity);

            CUnit* unit = unitHandler.GetUnit(unitId.value);
            if (unit == nullptr) {
                LOG_L(L_ERROR, "[%s][f=%d] BipedAnimMoveType entity has stale unit id %i (GetUnit returned null)", __func__, gs->frameNum, unitId.value);
                return;
            }
            CBipedAnimMoveType* moveType = static_cast<CBipedAnimMoveType*>(unit->moveType);
            assert(moveType != nullptr);
            LOG_L(L_WARNING, "[%s][f=%d] phase1 job starting for unit %i", __func__, gs->frameNum, unitId.value);

            #ifndef NDEBUG
            unit->SanityCheck();
            #endif

            // path/turn intent (inherited, unrelated to anchors) ...
            moveType->UpdateTraversalPlan();
            LOG_L(L_WARNING, "[%s][f=%d] UpdateTraversalPlan returned for unit %i", __func__, gs->frameNum, unitId.value);
            // ... and this frame's anchor read (piece visibility already
            // set by the script's reaction to last frame's CURRENT_SPEED).
            moveType->UpdateAnchorMotion();
            LOG_L(L_WARNING, "[%s][f=%d] UpdateAnchorMotion returned for unit %i", __func__, gs->frameNum, unitId.value);
        });
    }
    {
        SCOPED_TIMER("Sim::Unit::MoveType::Biped::2::UpdatePreCollisions");

        // ST due to the numerous synced vars being changed. Filtered by the
        // BipedAnimMoveType tag: GroundMoveSystem handles the same event
        // component type for its own (differently-tagged) entities.
        {
            auto view = Sim::registry.view<BipedAnimMoveType, ChangeHeadingEvent>();
            view.each([](BipedAnimMoveType& unitId, ChangeHeadingEvent& event){
                if (event.changed) {
                    CUnit* unit = unitHandler.GetUnit(unitId.value);
                    if (unit == nullptr) { event.changed = false; return; }
                    CBipedAnimMoveType* moveType = static_cast<CBipedAnimMoveType*>(unit->moveType);
                    moveType->ChangeHeading(event.deltaHeading);
                    event.changed = false;
                }
            });
        }
        {
            auto view = Sim::registry.view<BipedAnimMoveType, ChangeMainHeadingEvent>();
            view.each([](BipedAnimMoveType& unitId, ChangeMainHeadingEvent& event){
                if (event.changed) {
                    CUnit* unit = unitHandler.GetUnit(unitId.value);
                    if (unit == nullptr) { event.changed = false; return; }
                    CBipedAnimMoveType* moveType = static_cast<CBipedAnimMoveType*>(unit->moveType);
                    moveType->SetMainHeading();
                    event.changed = false;
                }
            });
        }

        auto view = Sim::registry.view<BipedAnimMoveType>();
        for_mt(0, view.size(), [&view](const int i){
            auto entity = view.storage<BipedAnimMoveType>()[i];
            auto unitId = view.get<BipedAnimMoveType>(entity);

            CUnit* unit = unitHandler.GetUnit(unitId.value);
            if (unit == nullptr) return;
            CBipedAnimMoveType* moveType = static_cast<CBipedAnimMoveType*>(unit->moveType);
            assert(moveType != nullptr);

            // legality-filters animMoveRequest (from UpdateAnchorMotion) via
            // UpdatePos() and stages the result in resultantForces, exactly
            // like CGroundMoveType::UpdateUnitPosition() does.
            moveType->UpdateAnimUnitPosition();
        });

        view.each([](BipedAnimMoveType& unitId){
            CUnit* unit = unitHandler.GetUnit(unitId.value);
            if (unit == nullptr) return;
            CBipedAnimMoveType* moveType = static_cast<CBipedAnimMoveType*>(unit->moveType);
            assert(moveType != nullptr);

            // inherited unchanged: commits resultantForces via owner->Move(),
            // handles arrived/fail/skid/falling bookkeeping.
            moveType->UpdatePreCollisions();

            if (!unit->pos.IsInBounds() && (unit->speed.w > MAX_UNIT_SPEED))
                unit->ForcedKillUnit(nullptr, false, true, -CSolidObject::DAMAGE_KILLED_OOB);
        });
    }
    {
        SCOPED_TIMER("Sim::Unit::MoveType::Biped::3::CollisionDetection");
        auto view = Sim::registry.view<BipedAnimMoveType>();
        for_mt(0, view.size(), [&view](const int i){
            auto entity = view.storage<BipedAnimMoveType>()[i];
            assert( Sim::registry.valid(entity) );
            assert( Sim::registry.all_of<BipedAnimMoveType>(entity) );

            auto unitId = view.get<BipedAnimMoveType>(entity);

            CUnit* unit = unitHandler.GetUnit(unitId.value);
            if (unit == nullptr) return;
            CBipedAnimMoveType* moveType = static_cast<CBipedAnimMoveType*>(unit->moveType);
            assert(moveType != nullptr);

            moveType->SetMtJobId(i);
            // inherited unchanged: unit-unit/feature push forces.
            moveType->UpdateCollisionDetections();
        });
    }
    {
        SCOPED_TIMER("Sim::Unit::MoveType::Biped::4::ProcessCollisionEvents");

        issue_events<UnitCrushEvents>([](const UnitCrushEvent& event) {
            event.collidee->Kill(event.collider, event.crushImpulse, true);
        });
        issue_events<FeatureCrushEvents>([](const FeatureCrushEvent& event) {
            event.collidee->Kill(event.collider, event.crushImpulse, true);
        });
        issue_events<UnitCollisionEvents>([&](const UnitCollisionEvent& event) {
            eventHandler.UnitUnitCollision(event.collider, event.collidee);
        });
        issue_events<FeatureCollisionEvents>([](const FeatureCollisionEvent& event) {
            eventHandler.UnitFeatureCollision(event.collider, event.collidee);
        });
        issue_events<FeatureMoveEvents>([](const FeatureMoveEvent& event) {
            quadField.RemoveFeature(event.collidee);
            event.collidee->Move(event.moveImpulse, true);
            quadField.AddFeature(event.collidee);
        });
    }
    {
        // TODO: the vars are synced and that's what is stopping this being MT'ed.
        SCOPED_TIMER("Sim::Unit::MoveType::Biped::5::Update");
        auto view = Sim::registry.view<BipedAnimMoveType>();
        view.each([&view](BipedAnimMoveType& unitId){
            CUnit* unit = unitHandler.GetUnit(unitId.value);
            if (unit == nullptr) return;
            CBipedAnimMoveType* moveType = static_cast<CBipedAnimMoveType*>(unit->moveType);
            assert(moveType != nullptr);

            // commits collision push and feeds back realized speed
            // (StartMoving/StopMoving, push-resistance bookkeeping).
            if (moveType->Update())
                eventHandler.UnitMoved(unit);

            #ifndef NDEBUG
            unit->SanityCheck();
            #endif
        });
    }
}

void BipedAnimMoveSystem::Shutdown() {}
