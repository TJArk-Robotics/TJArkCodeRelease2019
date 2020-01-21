#pragma once

#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Blackboard.h"

class SpecialActionsBase
{
public:
    /* REQUIRES */
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    const LegMotionSelection &theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
    const StiffnessSettings &theStiffnessSettings = Blackboard::getInstance().stiffnessSettings;

    /* PROVIDES */
    // SpecialActionsOutput &_theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
};

class SpecialActions : public SpecialActionsBase
{
public:
    /**
     * Represents a node of the motion net.
     * The motion net is organised in an array of nodes (MotionNetNode).
     */
    class MotionNetNode
    {
    public:
        enum NodeType
        {
            typeConditionalTransition, /**< The current node is a conditional transition. */
            typeTransition,            /**< The current node is a transition. */
            typeData,                  /**< The current node is a motor data vector. */
            typeStiffness,             /**< The current node is a motor stiffness tuple. */
            NumOfNodeType
        };

        float d[Joints::NumOfJoint + 4]; /**< Represent a set of values from a data line. */
                                         /*
        //possible content:
        {typeData, d[0]..d[21], interpolationMode, dataRepetitionCounter, execMotionRequest}
        {typeConditionalTransition, to_motion, via_label, 17*0, execMotionRequest}
        {typeTransition, to_label, 18*0, execMotionRequest}
        */

        void toJointRequest(JointRequest &jointRequest, int &dataRepetitionCounter, bool &interpolationMode, bool &deShakeMode) const
        {
            for (int i = 0; i < Joints::NumOfJoint; i++)
            {
                jointRequest.angles[i] = static_cast<float>(d[i + 1]);
            }
            interpolationMode = (static_cast<int>(d[Joints::NumOfJoint + 1]) & 1) != 0;
            deShakeMode = (static_cast<int>(d[Joints::NumOfJoint + 1]) & 2) != 0;
            dataRepetitionCounter = static_cast<int>(d[Joints::NumOfJoint + 2]);
        }

        void toStiffnessRequest(StiffnessData &stiffnessRequest, int &stiffnessInterpolationTime)
        {
            for (int i = 0; i < Joints::NumOfJoint; i++)
            {
                stiffnessRequest.stiffnesses[i] = static_cast<int>(d[i + 1]);
            }
            stiffnessInterpolationTime = static_cast<int>(d[Joints::NumOfJoint + 1]);
        }

        SpecialActionRequest::SpecialActionID getSpecialActionID() const
        {
            return SpecialActionRequest::SpecialActionID(short(d[Joints::NumOfJoint + 3]));
        }
    };

    /**
     * MotionNetData encapsulates all the motion data in the motion net.
     */
    class MotionNetData
    {
    public:
        /** Default constructor. */
        MotionNetData() : nodeArray(0) {}

        /** Destructor. */
        ~MotionNetData()
        {
            if (nodeArray)
                delete[] nodeArray;
        }

        /** Loads the motion net. */
        void load(std::vector<float> &motionData);

        /** jump table from extern.mof: get start index from request */
        short label_extern_start[SpecialActionRequest::NumOfSpecialActionID + 1];

        /** The motion net */
        MotionNetNode *nodeArray;
    };

    class SpecialActionInfo
    {
    public:
        enum OdometryType
        {
            none,       /**< No odometry, means no movement. */
            once,       /**< Odometry pose is used once the motion is executed. */
            homogeneous, /**< Odometry pose describes speed and is used each tick. */
            NumOfOdometryType
        };
        SpecialActionRequest::SpecialActionID id; /**< The id to which belongs this SpecialActionInfo. */
        OdometryType type;                        /**< The type of this odometry entry. */
        Pose2f odometryOffset;                    /**< The displacement performed by the special action. */
        bool isMotionStable;                      /**< Is the position of the camera directly related to the kinematic chain of joint angles? */
        SpecialActionInfo() : type(none), isMotionStable(false) {}
        SpecialActionInfo(SpecialActionRequest::SpecialActionID id, OdometryType type, bool isMotionStable) : id(id), type(type), isMotionStable(isMotionStable) {}
    };

    class OdometryParams
    {
    public:
        std::vector<SpecialActionInfo> specialActionInfos;
    };

    StiffnessData currentStiffnessRequest; /**< The current stiffness of the joints */
    StiffnessData lastStiffnessRequest;    /**< The last stiffness data*/

    bool wasEndOfSpecialAction;                                                  /**< Was the SpecialAction at the end in the last frame? */
    int stiffnessInterpolationCounter;                                           /**< Cycle counter for current stiffness interpolation */
    int stiffnessInterpolationLength;                                            /**< Length of the current stiffness interpolation */
    static thread_local SpecialActions *theInstance;                             /**< Points to the only instance of this class in this process or is 0 if there is none. */
    bool wasActive;                                                              /**< Was this module active in the previous frame? */
    MotionNetData motionNetData;                                                 /**< The motion data array. */
    short currentNode;                                                           /**< Current motion net node */
    JointRequest currentRequest;                                                 /**< Current joint data. */
    JointRequest lastRequest;                                                    /**< Last data for interpolation. */
    bool interpolationMode;                                                      /**< True if values should be interpolated. */
    bool deShakeMode;                                                            /**< True if shaking of arms should be prevented. */
    int dataRepetitionLength;                                                    /**< Length of current data line in cycles. */
    int dataRepetitionCounter;                                                   /**< Cycle counter for current data line. */
    SpecialActionInfo infoTable[SpecialActionRequest::NumOfSpecialActionID + 1]; /**< Odometry offset table. */
    SpecialActionInfo currentInfo;                                               /**< Information about the special action currently executed. */
    SpecialActionRequest::SpecialActionID lastSpecialAction;                     /**< type of last executed special action. */
    bool mirror;                                                                 /**< Mirror current special actions? */

    /**
     * Called from a MessageQueue to distribute messages.
     * @param message The message that can be read.
     * @return True if the message was handled.
     */
    // bool handleMessage2();

    /** Get next motion node from motion net */
    bool getNextData(const SpecialActionRequest &specialActionRequest, SpecialActionsOutput &specialActionsOutput);

    /** Calculates the next joint data vector by interpolating if necessary */
    void calculateJointRequest(JointRequest &jointRequest);

    void update();
    void update(SpecialActionsOutput &specialActionsOutput);

    SpecialActions();

    ~SpecialActions() { theInstance = nullptr; }
};