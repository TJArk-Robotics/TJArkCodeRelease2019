#include "SpecialActions.h"
#include "Tools/Motion/MofCompiler.h"
#include "Tools/Module/ModuleManager.h"

thread_local SpecialActions *SpecialActions::theInstance = nullptr;

void SpecialActions::MotionNetData::load(std::vector<float> &motionData)
{
    int dataCounter = 0;

    for (int i = 0; i < SpecialActionRequest::NumOfSpecialActionID; ++i)
        label_extern_start[i] = static_cast<short>(motionData[dataCounter++]);
    label_extern_start[SpecialActionRequest::NumOfSpecialActionID] = 0;

    int numberOfNodes = static_cast<int>(motionData[dataCounter++]);

    if (nodeArray)
        delete[] nodeArray;

    nodeArray = new MotionNetNode[numberOfNodes];

    for (int i = 0; i < numberOfNodes; ++i)
    {
        short s = static_cast<short>(motionData[dataCounter++]);

        switch (s)
        {
        case 2:
            nodeArray[i].d[0] = static_cast<short>(MotionNetNode::typeTransition);
            nodeArray[i].d[1] = motionData[dataCounter++];
            nodeArray[i].d[Joints::NumOfJoint + 3] = motionData[dataCounter++];
            break;
        case 1:
            nodeArray[i].d[0] = static_cast<short>(MotionNetNode::typeConditionalTransition);
            nodeArray[i].d[1] = motionData[dataCounter++];
            nodeArray[i].d[2] = motionData[dataCounter++];
            nodeArray[i].d[Joints::NumOfJoint + 3] = motionData[dataCounter++];
            break;
        case 4:
            nodeArray[i].d[0] = static_cast<short>(MotionNetNode::typeStiffness);
            for (int j = 1; j < Joints::NumOfJoint + 3; j++)
                nodeArray[i].d[j] = motionData[dataCounter++];
            break;
        case 3:
            nodeArray[i].d[0] = static_cast<short>(MotionNetNode::typeData);
            for (int j = 1; j < Joints::NumOfJoint + 1; ++j)
            {
                nodeArray[i].d[j] = motionData[dataCounter++];
                if (nodeArray[i].d[j] != JointAngles::off &&
                    nodeArray[i].d[j] != JointAngles::ignore)
                    nodeArray[i].d[j] = Angle::fromDegrees(nodeArray[i].d[j]);
            }
            for (int k = Joints::NumOfJoint + 1; k < Joints::NumOfJoint + 4; ++k)
                nodeArray[i].d[k] = motionData[dataCounter++];
            break;
        }
    }
}

SpecialActions::SpecialActions() : wasEndOfSpecialAction(false),
                                   //stiffnessInterpolationStart(0),
                                   stiffnessInterpolationCounter(0),
                                   stiffnessInterpolationLength(0),
                                   wasActive(false),
                                   dataRepetitionCounter(0),
                                   lastSpecialAction(SpecialActionRequest::NumOfSpecialActionID),
                                   mirror(false)
{
    theInstance = this;

    std::vector<float> motionData;
    char errorBuffer[10000];
    MofCompiler *mofCompiler = new MofCompiler;
    if (!mofCompiler->compileMofs(errorBuffer, sizeof(errorBuffer), motionData))
        std::cout << "[ERROR] Error while parsing mof files:" << std::endl;
    else
        motionNetData.load(motionData);
    delete mofCompiler;

    if (*errorBuffer)
        std::cout << "[ERROR] " << errorBuffer << std::endl;

    // create an uninitialised motion request to set startup motion
    currentNode = motionNetData.label_extern_start[SpecialActionRequest().specialAction];

    // read entries from file
    // TODO: read from configure file
    OdometryParams infos;
    infos.specialActionInfos.push_back(SpecialActionInfo(SpecialActionRequest::playDead, SpecialActionInfo::none, false));
    infos.specialActionInfos.push_back(SpecialActionInfo(SpecialActionRequest::standHigh, SpecialActionInfo::none, true));
    infos.specialActionInfos.push_back(SpecialActionInfo(SpecialActionRequest::standHighLookUp, SpecialActionInfo::none, true));

    for (std::vector<SpecialActionInfo>::const_iterator it = infos.specialActionInfos.begin(); it != infos.specialActionInfos.end(); it++)
    {
        infoTable[it->id] = SpecialActionInfo(*it);
        if (it->type == SpecialActionInfo::once || it->type == SpecialActionInfo::homogeneous)
        {
            infoTable[it->id].odometryOffset.rotation = it->odometryOffset.rotation;
            if (it->type == SpecialActionInfo::homogeneous)
            {
                // convert from mm/seconds to mm/tick
                constexpr float motionCycleTime = Constants::motionCycleTime;
                infoTable[it->type].odometryOffset.translation *= motionCycleTime;
                // convert from rad/seconds to rad/tick
                infoTable[it->type].odometryOffset.rotation *= motionCycleTime;
            }
        }
    }
}

bool SpecialActions::getNextData(const SpecialActionRequest &specialActionRequest, SpecialActionsOutput &specialActionsOutput)
{
    while (static_cast<MotionNetNode::NodeType>(short(motionNetData.nodeArray[currentNode].d[0])) != MotionNetNode::typeData)
    {
        switch (static_cast<MotionNetNode::NodeType>(short(motionNetData.nodeArray[currentNode].d[0])))
        {
        case MotionNetNode::typeStiffness:
            lastStiffnessRequest = specialActionsOutput.stiffnessData; //currentStiffnessRequest;
            motionNetData.nodeArray[currentNode].toStiffnessRequest(currentStiffnessRequest, stiffnessInterpolationLength);
            stiffnessInterpolationCounter = stiffnessInterpolationLength;
            currentNode++;
            break;
        case MotionNetNode::typeConditionalTransition:
            if (motionNetData.nodeArray[currentNode].d[2] != static_cast<short>(specialActionRequest.specialAction))
            {
                currentNode++;
                break;
            }
            //no break here: if condition is true, continue with transition!
        case MotionNetNode::typeTransition:
            // follow transition
            if (currentNode == 0) //we come from extern
                currentNode = motionNetData.label_extern_start[static_cast<short>(specialActionRequest.specialAction)];
            else
                currentNode = static_cast<short>(motionNetData.nodeArray[currentNode].d[1]);
            mirror = specialActionRequest.mirror;
            // leave if transition to external motion
            if (currentNode == 0)
                return false;
            break;
        case MotionNetNode::typeData:
            break;
        }
    }

    motionNetData.nodeArray[currentNode].toJointRequest(currentRequest, dataRepetitionLength, interpolationMode, deShakeMode);
    dataRepetitionCounter = dataRepetitionLength;

    specialActionsOutput.executedSpecialAction.specialAction = motionNetData.nodeArray[currentNode++].getSpecialActionID();
    specialActionsOutput.executedSpecialAction.mirror = mirror;
    specialActionsOutput.isMotionStable = infoTable[specialActionsOutput.executedSpecialAction.specialAction].isMotionStable;

    //get currently executed special action from motion net traversal:
    if (specialActionsOutput.executedSpecialAction.specialAction != lastSpecialAction)
    {
        currentInfo = infoTable[specialActionsOutput.executedSpecialAction.specialAction];
        lastSpecialAction = specialActionsOutput.executedSpecialAction.specialAction;
    }

    return true;
}

void SpecialActions::calculateJointRequest(JointRequest &jointRequest)
{
    float ratio, f, t;

    //joint angles
    if (interpolationMode)
    {
        ratio = dataRepetitionCounter / static_cast<float>(dataRepetitionLength);
        for (int i = 0; i < Joints::NumOfJoint; ++i)
        {
            f = lastRequest.angles[i];
            if (!mirror)
                t = currentRequest.angles[i];
            else
                t = currentRequest.mirror(static_cast<Joints::Joint>(i));
            // if fromAngle is off or ignore use JointAngles for further calculation
            if (f == JointAngles::off || f == JointAngles::ignore)
                f = theJointAngles.angles[i];

            // if toAngle is off or ignore -> turn joint off/ignore
            if (t == JointAngles::off || t == JointAngles::ignore)
                jointRequest.angles[i] = t;
            //interpolate
            else
                jointRequest.angles[i] = static_cast<float>(t + (f - t) * ratio);
        }
    }
    else
    {
        if (!mirror)
            jointRequest = currentRequest;
        else
            jointRequest.mirror(currentRequest);
    }

    //stiffness stuff
    if (stiffnessInterpolationCounter <= 0)
    {
        if (!mirror)
            jointRequest.stiffnessData = currentStiffnessRequest;
        else
            jointRequest.stiffnessData.mirror(currentStiffnessRequest);
    }
    else
    {
        ratio = static_cast<float>(stiffnessInterpolationCounter) / stiffnessInterpolationLength;
        int f, t;
        for (int i = 0; i < Joints::NumOfJoint; i++)
        {
            f = lastStiffnessRequest.stiffnesses[i];
            if (!mirror)
                t = currentStiffnessRequest.stiffnesses[i];
            else
                t = currentStiffnessRequest.mirror(static_cast<Joints::Joint>(i));
            if (t == f)
                jointRequest.stiffnessData.stiffnesses[i] = t;
            else
            {
                if (f == StiffnessData::useDefault)
                    f = theStiffnessSettings.stiffnesses[i];
                if (t == StiffnessData::useDefault)
                    t = mirror ? theStiffnessSettings.mirror(static_cast<Joints::Joint>(i)) : theStiffnessSettings.stiffnesses[i];
                jointRequest.stiffnessData.stiffnesses[i] = int(float(t) + float(f - t) * ratio);
            }
        }
    }
}

void SpecialActions::update()
{
    // JointAngles
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }
    // LegMotionSelection
    if (!Blackboard::getInstance().updatedMap["LegMotionSelection"])
    {
        LegMotionSelection &_theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
        ModuleManager::theInstance->motionSelector.update(_theLegMotionSelection);
        Blackboard::getInstance().updatedMap["LegMotionSelection"] = true;
    }
    // StiffnessSettings
}

void SpecialActions::update(SpecialActionsOutput &specialActionsOutput)
{
    update();

    if (!motionNetData.nodeArray)
    {
        specialActionsOutput.angles.fill(0);
        specialActionsOutput.stiffnessData.stiffnesses.fill(0);
        return;
    }

    float speedFactor = 1.0f;
    if (theLegMotionSelection.specialActionMode != LegMotionSelection::deactive)
    {
        specialActionsOutput.isLeavingPossible = true;
        if (dataRepetitionCounter <= 0)
        {
            if (!wasActive)
            {
                // entered from external motion
                currentNode = 0;
                for (int i = 0; i < Joints::NumOfJoint; i++)
                    lastRequest.angles[i] = theJointAngles.angles[i];
                lastSpecialAction = SpecialActionRequest::NumOfSpecialActionID;
            }

            // this is need with a special actions gets executed directly after another without
            // switching to a different motion for interpolating the stiffness
            if (wasEndOfSpecialAction)
            {
                specialActionsOutput.stiffnessData.resetToDefault();
                if (!mirror)
                    lastStiffnessRequest = currentStiffnessRequest;
                else
                    lastStiffnessRequest.mirror(currentStiffnessRequest);
                currentStiffnessRequest.resetToDefault();
            }
            wasEndOfSpecialAction = false;
            // search next data, leave on transition to external motion
            // std::cout << "theLegMotionSelection.specialActionRequest:" << theLegMotionSelection.specialActionRequest.specialAction << std::endl;
            if (!getNextData(theLegMotionSelection.specialActionRequest, specialActionsOutput))
            {
                wasActive = true;
                wasEndOfSpecialAction = true;
                specialActionsOutput.odometryOffset = Pose2f();
                return;
            }
        }
        else
        {
            dataRepetitionCounter -= int(Constants::motionCycleTime * 1000 * speedFactor);
            stiffnessInterpolationCounter -= int(Constants::motionCycleTime * 1000 * speedFactor);
        }

        // set current joint values
        calculateJointRequest(specialActionsOutput);

        // odometry update
        if (currentInfo.type == SpecialActionInfo::homogeneous || currentInfo.type == SpecialActionInfo::once)
            if (mirror)
                specialActionsOutput.odometryOffset = Pose2f(-currentInfo.odometryOffset.rotation, currentInfo.odometryOffset.translation.x(), -currentInfo.odometryOffset.translation.y());
            else
                specialActionsOutput.odometryOffset = currentInfo.odometryOffset;
        else
            specialActionsOutput.odometryOffset = Pose2f();
        if (currentInfo.type == SpecialActionInfo::once)
            currentInfo.type = SpecialActionInfo::none;

        // store value if current data line finished
        if (dataRepetitionCounter <= 0)
        {
            if (!mirror)
                lastRequest = currentRequest;
            else
                lastRequest.mirror(currentRequest);
        }
        specialActionsOutput.isLeavingPossible = false;
        if (deShakeMode)
            for (int i = Joints::lShoulderPitch; i <= Joints::rElbowRoll; i++)
                if (Random::bernoulli(0.25))
                    specialActionsOutput.angles[i] = JointAngles::off;
    }
    else
    {
        dataRepetitionCounter = 0;
        wasEndOfSpecialAction = true;
    }
    wasActive = theLegMotionSelection.specialActionMode != LegMotionSelection::deactive;
}
