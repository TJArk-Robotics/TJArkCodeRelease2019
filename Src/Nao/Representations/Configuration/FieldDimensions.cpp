#include "FieldDimensions.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Settings.h"
#include <algorithm>

void FieldDimensions::load()
{
    xPosOpponentFieldBorder = 5200; // origin -> far field boundary
    xPosOpponentGoal = 5055;        // origin -> middle far side of goal
    xPosOpponentGoalPost = 4525;    // origin -> middle goal post
    xPosOpponentGroundline = 4500;  // origin -> middle goal line
    xPosOpponentPenaltyArea = 3900; // origin -> middle near penalty area line
    xPosOpponentDropInLine = 3500;  // origin -> far end of throw-in line
    xPosOpponentPenaltyMark = 3200; // origin -> middle penalty mark
    xPosPenaltyStrikerStartPosition = 2200;
    xPosHalfWayLine = 0;
    xPosOwnPenaltyMark = -xPosOpponentPenaltyMark;
    xPosOwnDropInLine = -xPosOpponentDropInLine;
    xPosOwnPenaltyArea = -xPosOpponentPenaltyArea;
    xPosOwnGroundline = -xPosOpponentGroundline;
    xPosOwnGoalPost = -xPosOpponentGoalPost;
    xPosOwnGoal = -xPosOpponentGoal;
    xPosOwnFieldBorder = -xPosOpponentFieldBorder;

    yPosLeftFieldBorder = 3700; // origin -> side field boundary
    yPosLeftSideline = 3000;    // origin -> middle side line
    yPosLeftDropInLine = 2600;  // origin -> thow-in line
    yPosLeftPenaltyArea = 1100; // origin -> middle left penalty area line
    yPosLeftGoal = 800;         // origin -> middle left goal post
    yPosCenterGoal = 0;
    yPosRightGoal = -yPosLeftGoal;
    yPosRightPenaltyArea = -yPosLeftPenaltyArea;
    yPosRightDropInLine = -yPosLeftDropInLine;
    yPosRightSideline = -yPosLeftSideline;
    yPosRightFieldBorder = -yPosLeftFieldBorder;

    fieldLinesWidth = 50;
    centerCircleRadius = 750;
    goalPostRadius = 50;
    crossBarRadius = goalPostRadius;
    goalHeight = 900;
    penaltyMarkSize = 100;

    /**< Goal Frame Lines*/
    goalFrameLines.push(Vector2f(xPosOwnGoalPost, yPosLeftGoal), Vector2f(xPosOwnGoal, yPosLeftGoal), false);
    goalFrameLines.push(Vector2f(xPosOwnGoal, yPosLeftGoal), Vector2f(xPosOwnGoal, yPosRightGoal), false);
    goalFrameLines.push(Vector2f(xPosOwnGoalPost, yPosRightGoal), Vector2f(xPosOwnGoal, yPosRightGoal), false);
    goalFrameLines.push(Vector2f(xPosOpponentGoalPost, yPosLeftGoal), Vector2f(xPosOpponentGoal, yPosLeftGoal), false);
    goalFrameLines.push(Vector2f(xPosOpponentGoal, yPosLeftGoal), Vector2f(xPosOpponentGoal, yPosRightGoal), false);
    goalFrameLines.push(Vector2f(xPosOpponentGoalPost, yPosRightGoal), Vector2f(xPosOpponentGoal, yPosRightGoal), false);

    /**< Field Lines*/
    // field border lines
    fieldLines.push(Vector2f(xPosOpponentGroundline, yPosRightSideline), Vector2f(xPosOpponentGroundline, yPosLeftSideline), false);
    fieldLines.push(Vector2f(xPosOpponentGroundline, yPosLeftSideline), Vector2f(xPosOwnGroundline, yPosLeftSideline), false);
    fieldLines.push(Vector2f(xPosOwnGroundline, yPosLeftSideline), Vector2f(xPosOwnGroundline, yPosRightSideline), false);
    fieldLines.push(Vector2f(xPosOwnGroundline, yPosRightSideline), Vector2f(xPosOpponentGroundline, yPosRightSideline), false);
    // center line
    fieldLines.push(Vector2f(xPosHalfWayLine, yPosLeftSideline), Vector2f(xPosHalfWayLine, yPosRightSideline), false);

    // penalty areas
    fieldLines.push(Vector2f(xPosOwnGroundline, yPosLeftPenaltyArea), Vector2f(xPosOwnPenaltyArea, yPosLeftPenaltyArea), false);
    fieldLines.push(Vector2f(xPosOwnPenaltyArea, yPosLeftPenaltyArea), Vector2f(xPosOwnPenaltyArea, yPosRightPenaltyArea), false);
    fieldLines.push(Vector2f(xPosOwnPenaltyArea, yPosRightPenaltyArea), Vector2f(xPosOwnGroundline, yPosRightPenaltyArea), false);
    fieldLines.push(Vector2f(xPosOpponentGroundline, yPosLeftPenaltyArea), Vector2f(xPosOpponentPenaltyArea, yPosLeftPenaltyArea), false);
    fieldLines.push(Vector2f(xPosOpponentPenaltyArea, yPosLeftPenaltyArea), Vector2f(xPosOpponentPenaltyArea, yPosRightPenaltyArea), false);
    fieldLines.push(Vector2f(xPosOpponentPenaltyArea, yPosRightPenaltyArea), Vector2f(xPosOpponentGroundline, yPosRightPenaltyArea), false);

    // penalty and center marks
    fieldLines.push(Vector2f(3150, 0), Vector2f(3250, 0), false);
    fieldLines.push(Vector2f(xPosOpponentPenaltyMark, -fieldLinesWidth), Vector2f(xPosOpponentPenaltyMark, fieldLinesWidth), false);
    fieldLines.push(Vector2f(-3150, 0), Vector2f(-3250, 0), false);
    fieldLines.push(Vector2f(xPosOwnPenaltyMark, -fieldLinesWidth), Vector2f(xPosOwnPenaltyMark, fieldLinesWidth), false);
    fieldLines.push(Vector2f(-fieldLinesWidth, 0), Vector2f(fieldLinesWidth, 0), false);

    /**< Center Circle */
    centerCircle.center = Vector2f();
    centerCircle.radius = centerCircleRadius;
    centerCircle.numOfSegments = 16;

    /**< Corners */
    // xCorner
    corners[xCorner].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[xCorner].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    // tCorner0
    corners[tCorner0].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner0].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner0].push_back(Vector2f(xPosOwnGroundline, yPosLeftPenaltyArea));
    corners[tCorner0].push_back(Vector2f(xPosOwnGroundline, yPosRightPenaltyArea));
    // tCorner90
    corners[tCorner90].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner90].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner90].push_back(Vector2f(xPosHalfWayLine, yPosRightSideline));
    // tCorner180
    corners[tCorner180].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner180].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner180].push_back(Vector2f(xPosOpponentGroundline, yPosLeftPenaltyArea));
    corners[tCorner180].push_back(Vector2f(xPosOpponentGroundline, yPosRightPenaltyArea));
    // tCorner270
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, yPosLeftSideline));
    // lCorner0
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosOwnGroundline, yPosLeftPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosOwnGroundline, yPosRightPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, yPosRightSideline));
    corners[tCorner270].push_back(Vector2f(xPosOwnGroundline, yPosRightSideline));
    corners[tCorner270].push_back(Vector2f(xPosOpponentPenaltyArea, yPosRightPenaltyArea));
    // lCorner90
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosOpponentGroundline, yPosLeftPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosOpponentGroundline, yPosRightPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, yPosRightSideline));
    corners[tCorner270].push_back(Vector2f(xPosOpponentGroundline, yPosRightSideline));
    corners[tCorner270].push_back(Vector2f(xPosOwnPenaltyArea, yPosRightPenaltyArea));
    // lCorner180
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosOpponentGroundline, yPosLeftPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosOpponentGroundline, yPosRightPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, yPosLeftSideline));
    corners[tCorner270].push_back(Vector2f(xPosOpponentGroundline, yPosLeftSideline));
    corners[tCorner270].push_back(Vector2f(xPosOwnPenaltyArea, yPosLeftPenaltyArea));
    // lCorner270
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, -centerCircleRadius));
    corners[tCorner270].push_back(Vector2f(xPosOwnGroundline, yPosLeftPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosOwnGroundline, yPosRightPenaltyArea));
    corners[tCorner270].push_back(Vector2f(xPosHalfWayLine, yPosLeftSideline));
    corners[tCorner270].push_back(Vector2f(xPosOwnGroundline, yPosLeftSideline));
    corners[tCorner270].push_back(Vector2f(xPosOpponentPenaltyArea, yPosLeftPenaltyArea));
}

Pose2f FieldDimensions::randomPoseOnField() const
{
    Pose2f pose;
    do
        pose = Pose2f::random(Rangef(-pi, pi), boundary.x, boundary.y);
    while (!isInsideField(pose.translation));
    return pose;
}

Pose2f FieldDimensions::randomPoseOnCarpet() const
{
    Pose2f pose;
    do
        pose = Pose2f::random(Rangef(-pi, pi), boundary.x, boundary.y);
    while (!isInsideCarpet(pose.translation));
    return pose;
}

void FieldDimensions::LinesTable::push(const Vector2f &from, const Vector2f &to, bool isPartOfCircle)
{
    LinesTable::Line line;
    line.from = from;
    line.to = to;
    line.isPartOfCircle = isPartOfCircle;
    lines.push_back(line);
}

void FieldDimensions::LinesTable::pushCircle(const Vector2f &center, float radius, int numOfSegments)
{
    Vector2f p1, p2;
    for (float a = 0; a <= pi_4; a += pi2 / numOfSegments)
    {
        p1 = Vector2f(sin(a), cos(a)) * radius;
        if (a > 0)
        {
            push(center + p1, center + p2, true);
            push(center + Vector2f(p1.x(), -p1.y()), center + Vector2f(p2.x(), -p2.y()), true);
            push(center + Vector2f(-p1.x(), p1.y()), center + Vector2f(-p2.x(), p2.y()), true);
            push(center - p1, center - p2, true);
            push(center + Vector2f(p1.y(), p1.x()), center + Vector2f(p2.y(), p2.x()), true);
            push(center + Vector2f(p1.y(), -p1.x()), center + Vector2f(p2.y(), -p2.x()), true);
            push(center + Vector2f(-p1.y(), p1.x()), center + Vector2f(-p2.y(), p2.x()), true);
            push(center + Vector2f(-p1.y(), -p1.x()), center + Vector2f(-p2.y(), -p2.x()), true);
        }
        p2 = p1;
    }
}

float FieldDimensions::LinesTable::getDistance(const Pose2f &pose) const
{
    float minDist = 100000;
    for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
        if (i->from.y() < 0 && i->to.y() > 0)
        {
            const float dist = i->from.x() + (i->to.x() - i->from.x()) * -i->from.y() / (i->to.y() - i->from.y());
            if (dist >= 0 && dist < minDist)
                minDist = dist;
        }
    return minDist == 100000 ? -1 : minDist;
}