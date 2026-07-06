//=================================================================================================
//
// Physics constraints: the 7 Source constraint types mapped onto Box3D joints.
//
//=================================================================================================

#pragma once

#include "vbox_interface.h"

#include <cstring>
#include <functional>

class Box3DPhysicsObject;
class Box3DPhysicsEnvironment;
class Box3DPhysicsConstraintGroup;

// Source constraint type + original params, so save/restore rebuilds via the same CreateXConstraint path.
enum Box3DConstraintKind
{
    kBox3DConstraint_None = 0,
    kBox3DConstraint_Ragdoll,
    kBox3DConstraint_Hinge,
    kBox3DConstraint_Fixed,
    kBox3DConstraint_Sliding,
    kBox3DConstraint_Ballsocket,
    kBox3DConstraint_Pulley,
    kBox3DConstraint_Length,
};

union Box3DConstraintParams
{
    // Members carry default initializers, so the union needs explicit no-op special members; it is only ever
    // filled as a byte blob.
    Box3DConstraintParams()
    {
    }
    ~Box3DConstraintParams()
    {
    }
    constraint_ragdollparams_t ragdoll;
    constraint_hingeparams_t hinge;
    constraint_fixedparams_t fixed;
    constraint_slidingparams_t sliding;
    constraint_ballsocketparams_t ballsocket;
    constraint_pulleyparams_t pulley;
    constraint_lengthparams_t length;
};

// Box3D joints have no enable flag, so Activate/Deactivate rebuild/destroy the joint via a stored closure.
class Box3DPhysicsConstraint final : public IPhysicsConstraint
{
public:
    Box3DPhysicsConstraint(
        Box3DPhysicsEnvironment* pEnvironment, Box3DPhysicsObject* pReference, Box3DPhysicsObject* pAttached);
    ~Box3DPhysicsConstraint() override;

    void Activate() override;
    void Deactivate() override;
    void SetGameData(void* gameData) override
    {
        m_pGameData = gameData;
    }
    void* GetGameData() const override
    {
        return m_pGameData;
    }
    IPhysicsObject* GetReferenceObject() const override;
    IPhysicsObject* GetAttachedObject() const override;
    void SetLinearMotor(float speed, float maxLinearImpulse) override;
    void SetAngularMotor(float rotSpeed, float maxAngularImpulse) override;
    void UpdateRagdollTransforms(const matrix3x4_t&, const matrix3x4_t&) override
    {
    }
    bool GetConstraintTransform(matrix3x4_t* pConstraintToReference, matrix3x4_t* pConstraintToAttached) const override;
    bool GetConstraintParams(constraint_breakableparams_t* pParams) const override;
    void OutputDebugInfo() override
    {
    }

    // Store the joint builder and create it now if bActive (grouped constraints defer to the group's Activate).
    void Init(const std::function<b3JointId()>& buildFn, bool bActive);
    void SetGroup(Box3DPhysicsConstraintGroup* pGroup)
    {
        m_pGroup = pGroup;
    }
    Box3DPhysicsConstraintGroup* GetGroup() const
    {
        return m_pGroup;
    }
    b3JointId GetJointId() const
    {
        return m_JointId;
    }

    // A constrained object is being destroyed: break the joint and null the stale pointer. Returns true the
    // first time (so the environment fires ConstraintBroken once).
    bool NotifyObjectDestroyed(Box3DPhysicsObject* pObject);

    // The joint exceeded its force/torque limit: destroy it and mark the constraint broken (it stays broken).
    void OnBroken();
    void SetBreakParams(const constraint_breakableparams_t& params)
    {
        m_BreakParams = params;
    }

    // Remember the source constraint type + params so save/restore can rebuild it via CreateXConstraint.
    void SetSaveInfo(Box3DConstraintKind kind, const void* pParams, int nSize)
    {
        m_SaveKind = kind;
        memcpy(&m_SaveParams, pParams, (size_t)nSize);
    }
    Box3DConstraintKind GetSaveKind() const
    {
        return m_SaveKind;
    }
    bool IsBroken() const
    {
        return m_bBroken;
    }
    const Box3DConstraintParams& GetSaveParams() const
    {
        return m_SaveParams;
    }

    // Pulley: Box3D has no pulley joint, so it is solved as a per-step impulse constraint coupling the two
    // rope segments (|A-pulleyA| + gearRatio*|B-pulleyB| = totalLength). SolvePulley runs each step.
    void SetupPulley(const b3Vec3 pulleyWorld[2], const b3Vec3 localAttach[2], float totalLength, float gearRatio, bool rigid);
    void SolvePulley(float dt);
    bool IsPulley() const
    {
        return m_bPulley;
    }

private:
    void DestroyJoint();
    // Apply the breakable params to the live joint: force/torque break thresholds and constraint strength.
    void ApplyConstraintTuning();

    Box3DPhysicsEnvironment* m_pEnvironment;
    Box3DPhysicsObject* m_pReference;
    Box3DPhysicsObject* m_pAttached;
    Box3DPhysicsConstraintGroup* m_pGroup = nullptr;
    void* m_pGameData = nullptr;
    b3JointId m_JointId = b3_nullJointId;
    std::function<b3JointId()> m_BuildFn;
    constraint_breakableparams_t m_BreakParams = {};
    bool m_bBroken = false;

    bool m_bPulley = false;
    b3Vec3 m_PulleyWorld[2] = {}; // pulley pivots, world space (metres)
    b3Vec3 m_PulleyLocal[2] = {}; // attach points, body-origin-local (metres)
    float m_flPulleyTotalLength = 0.0f;
    float m_flPulleyGearRatio = 1.0f;
    bool m_bPulleyRigid = false;

    Box3DConstraintKind m_SaveKind = kBox3DConstraint_None;
    Box3DConstraintParams m_SaveParams{};
};

// A Source spring on a Box3D distance joint with a soft spring.
class Box3DPhysicsSpring final : public IPhysicsSpring
{
public:
    Box3DPhysicsSpring(
        Box3DPhysicsEnvironment* pEnvironment, Box3DPhysicsObject* pStart, Box3DPhysicsObject* pEnd,
        const springparams_t* pParams);
    ~Box3DPhysicsSpring() override;

    void GetEndpoints(Vector* worldPositionStart, Vector* worldPositionEnd) override;
    void SetSpringConstant(float flSpringConstant) override;
    void SetSpringDamping(float flSpringDamping) override;
    void SetSpringLength(float flSpringLength) override;
    IPhysicsObject* GetStartObject() override;
    IPhysicsObject* GetEndObject() override;

    void NotifyObjectDestroyed(Box3DPhysicsObject* pObject);

    void SetSaveParams(const springparams_t& params)
    {
        m_SaveParams = params;
    }
    const springparams_t& GetSaveParams() const
    {
        return m_SaveParams;
    }

private:
    void PushSpringSettings();

    Box3DPhysicsEnvironment* m_pEnvironment;
    Box3DPhysicsObject* m_pStart;
    Box3DPhysicsObject* m_pEnd;
    springparams_t m_SaveParams = {};
    b3JointId m_JointId = b3_nullJointId;
    b3Vec3 m_AnchorStart; // body-local anchor points, for GetEndpoints
    b3Vec3 m_AnchorEnd;
    float m_flConstant;
    float m_flDamping;
};

class Box3DPhysicsConstraintGroup final : public IPhysicsConstraintGroup
{
public:
    void Activate() override;
    bool IsInErrorState() override
    {
        return false;
    }
    void ClearErrorState() override
    {
    }
    void GetErrorParams(constraint_groupparams_t* pParams) override
    {
        if (pParams)
            *pParams = m_Params;
    }
    void SetErrorParams(const constraint_groupparams_t& params) override
    {
        m_Params = params;
    }
    void SolvePenetration(IPhysicsObject*, IPhysicsObject*) override
    {
    }

    void AddConstraint(Box3DPhysicsConstraint* pConstraint)
    {
        m_Constraints.AddToTail(pConstraint);
    }
    void RemoveConstraint(Box3DPhysicsConstraint* pConstraint)
    {
        m_Constraints.FindAndRemove(pConstraint);
    }

private:
    CUtlVector<Box3DPhysicsConstraint*> m_Constraints;
    constraint_groupparams_t m_Params = {};
};
