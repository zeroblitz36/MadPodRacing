//Gold League 3rd place, score of 40.78
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <complex>
#include <memory>
#include <optional>

static constexpr float PI_FLOAT = 3.14159265359f;

struct Vec2 {
    float x = 0;
    float y = 0;
    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}

    float DistanceTo(const Vec2& p) const {
        return sqrtf((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
    }

    bool operator==(const Vec2& o) const {
        return x == o.x && y == o.y;
    }

    bool operator!=(const Vec2& o) const {
        return x != o.x || y != o.y;
    }

    Vec2 operator+(const Vec2& o) const {
        return Vec2(x + o.x, y + o.y);
    }

    Vec2 operator-(const Vec2& o) const {
        return Vec2(x - o.x, y - o.y);
    }

    Vec2 operator*(float a) const {
        return Vec2(x * a, y * a);
    }

    friend std::ostream& operator<<(std::ostream& os, const Vec2& p) {
        return os << "[" << p.x << "," << p.y << "]";
    }

    friend std::istream& operator>>(std::istream& is, Vec2& p) {
        return is >> p.x >> p.y;
    }

    float GetMagnitude() const
    {
        return std::hypot(x, y);
    }

    float GetAngleInDegrees() const
    {
        const float radians = std::atan2(y, x);
        if (radians >= 0)
        {
            return 180 * radians / PI_FLOAT;
        }
        else
        {
            return 180 * (radians + PI_FLOAT) / PI_FLOAT + 180;
        }
    }

    bool IsInsideCircle(float cx, float cy, float r) const
    {
        return std::hypot(cx - x, cy - y) <= r;
    }
};

struct PodState
{
    Vec2 position;
    Vec2 velocity;
    int degrees;
    int nextCheckPointId;

    friend std::istream& operator>>(std::istream& is, PodState& p) {
        return is >> p.position >> p.velocity >> p.degrees >> p.nextCheckPointId;
    }
};

Vec2 closestPointToLine(Vec2 a, Vec2 b, Vec2 p) {
    Vec2 a_to_p = p - a;
    Vec2 a_to_b = b - a;
    float atb2 = a.DistanceTo(b);
    atb2 *= atb2;
    if (atb2 == 0) {
        return p;
    }
    float atp_dot_atb = a_to_p.x * a_to_b.x + a_to_p.y * a_to_b.y;
    float t = atp_dot_atb / atb2;

    return Vec2(a.x + a_to_b.x * t, a.y + a_to_b.y * t);
}

static constexpr float FRICTION = 0.85f;
static constexpr float POD_SIZE = 400.f;
static constexpr float CHECKPOINT_RADIUS = 600.f;

class Pod
{
public:
    Pod(const std::shared_ptr<std::vector<Vec2>>& raceCoordinates, const char* tag, bool doBoostOnFirstFrame = false, bool doShieldToBoostAlly = false)
        : m_SharedRaceCoordinates(raceCoordinates)
        , m_Tag(tag)
        , m_DoBoostOnFirstFrame(doBoostOnFirstFrame)
        , m_DoShieldToBoostAlly(doShieldToBoostAlly)
    {
        if (m_SharedRaceCoordinates)
        {
            m_LongestStretchCandidate = 0.f;
            const auto& checkpoints = *m_SharedRaceCoordinates;
            const int n = checkpoints.size();
            for (int i = 0; i < n; ++i)
            {
                const float aux = checkpoints[i].DistanceTo(checkpoints[(i + 1) % n]);
                if (aux > m_LongestStretchCandidate)
                {
                    m_LongestStretchCandidate = aux;
                }
            }

            m_LongestStretchCandidate *= 0.70f;
        }
    }


    void ReadNewStateFromCin()
    {
        std::cin >> m_CurrentState;
        if (m_LastCheckpointId != m_CurrentState.nextCheckPointId)
        {
            m_CheckpointCount++;
            m_LastCheckpointId = m_CurrentState.nextCheckPointId;
        }
    }

    void WriteCommandToCout()
    {
        if (m_NumberOfTurnsSinceLastShield != -1)
        {
            m_NumberOfTurnsSinceLastShield++;
        }
        if (m_SharedRaceCoordinates)
        {
            const float absoluteDegreesDiffToCurrentCheckpoint = GetAbsoluteDegreesTowardsCurrentCheckpoint();

            float absoluteDegrees = absoluteDegreesDiffToCurrentCheckpoint;
            /*
            std::cerr << "[" << tag << "] :" <<
                " absDegreeDiff=" << absoluteDegreesDiffToCheckpoint <<
                " thrust=" << thrust << std::endl;
            */

            const bool isGoingForTheHomeRun = m_CheckpointCount == (m_SharedRaceCoordinates->size() * 3);

            if (isGoingForTheHomeRun)
            {
                std::cerr << "[" << m_Tag << "] : Is going for the HomeRun!\n";
            }

            const Vec2 currentCheckpoint = GetCurrentCheckpointPosition();
            Vec2 outputPos = currentCheckpoint;

            const Vec2 currentPos = m_CurrentState.position;
            const Vec2 positionDelta = m_CurrentState.velocity * FRICTION;
            const float distanceTravelledOnPreviousFrame = positionDelta.GetMagnitude();
            const Vec2 futurePos = currentPos + positionDelta;

            if (!isGoingForTheHomeRun && IsGoingToEnterCheckpointSoon())
            {
                outputPos = GetNextCheckpointPosition();
                absoluteDegrees = GetAbsoluteDegreesTowardsNextCheckpoint();
                std::cerr << "[" << m_Tag << "] :" << " drift towards next checkpoint, absoluteDegrees=" << absoluteDegrees << "\n";
            }
            else if (distanceTravelledOnPreviousFrame > 50 &&
                absoluteDegrees < 70.f &&
                futurePos.DistanceTo(currentCheckpoint) < currentPos.DistanceTo(currentCheckpoint))
            {
                Vec2 aux = closestPointToLine(currentPos, currentCheckpoint, futurePos);
                Vec2 aux2 = aux + (aux - futurePos);
                outputPos = aux2;
            }

            int thrust = 0;
            if (absoluteDegrees < 90)
            {
                thrust = (int)(std::ceil(100.f * std::cos(absoluteDegrees / 180.f)));
                std::cerr << "[" << m_Tag << "] :" << " thrust=" << thrust << " absoluteDegrees=" << absoluteDegrees << "\n";
            }

            if (m_IsBoostAvailable)
            {
                m_NumberOfFramesToWaitBeforeBoost--;
                if (m_DoBoostOnFirstFrame ||
                    (m_NumberOfFramesToWaitBeforeBoost < 0 && absoluteDegrees < 5.f && GetDistanceToCurrentCheckpoint() > m_LongestStretchCandidate))
                {
                    m_IsBoostAvailable = false;
                    std::cout << (int)(outputPos.x) << " " <<
                        (int)(outputPos.y) << " " <<
                        "BOOST" << std::endl;
                    return;
                }
            }


            if (m_NumberOfTurnsSinceLastShield == -1 || m_NumberOfTurnsSinceLastShield > 3)
            {
                if (ShouldEnableShieldForBeneficialCollisions())
                {
                    m_NumberOfTurnsSinceLastShield = 0;
                    std::cout << (int)(outputPos.x) << " " <<
                        (int)(outputPos.y) << " " <<
                        "SHIELD" << std::endl;
                    return;
                }
                /*
                if (IsAboutToCollideWithEnemyAndNoAlly())
                {
                    std::cerr << "[" << tag << "] :" <<
                    " DetectedCollision=SHIELDS UP\n";
                    m_NumberOfTurnsSinceLastShield = 0;
                    std::cout << (int)(outputPos.x) << " " <<
                        (int)(outputPos.y) << " " <<
                        "SHIELD" << std::endl;
                    return;
                }
                */
            }

            std::cout << (int)(outputPos.x) << " " <<
                (int)(outputPos.y) << " " <<
                thrust << std::endl;
        }
        else
        {
            //Why are we requesting a command for an enemy ?
            std::cout << 0 <<
                " " << 0 <<
                " " << 100 << std::endl;
        }
    }

    void DebugPrintCurrentState() const
    {
        std::cerr << "[" << m_Tag << "] :" <<
            " pos=" << m_CurrentState.position <<
            //" vel=" << m_CurrentState.velocity << 
            //" deg=" << m_CurrentState.degrees << 
            " chk=" << m_CurrentState.nextCheckPointId <<
            " boost=" << m_IsBoostAvailable <<
            " distChk=" << m_CurrentState.position.DistanceTo(GetCurrentCheckpointPosition()) <<
            std::endl;
    }

    Vec2 GetCurrentCheckpointPosition() const
    {
        if (m_SharedRaceCoordinates)
        {
            const auto& coords = *m_SharedRaceCoordinates;
            return coords[m_CurrentState.nextCheckPointId];
        }
        return Vec2();
    }

    Vec2 GetNextCheckpointPosition() const
    {
        if (m_SharedRaceCoordinates)
        {
            const auto& coords = *m_SharedRaceCoordinates;
            return coords[(m_CurrentState.nextCheckPointId + 1) % coords.size()];
        }
        return Vec2();
    }

    Vec2 GetVectorToCurrentCheckpoint() const
    {
        if (m_SharedRaceCoordinates)
        {
            const auto& coords = *m_SharedRaceCoordinates;
            return GetCurrentCheckpointPosition() - m_CurrentState.position;
        }
        return Vec2();
    }

    float GetDistanceToCurrentCheckpoint() const
    {
        if (m_SharedRaceCoordinates)
        {
            return GetVectorToCurrentCheckpoint().GetMagnitude();
        }
        return 0;
    }

    float GetCurrentVelocity() const
    {
        return m_CurrentState.velocity.GetMagnitude();
    }

    float GetAbsoluteDegreesTowardsCurrentCheckpoint()
    {
        const Vec2 vectorToCurrentCheckpoint = GetVectorToCurrentCheckpoint();
        const float degreesToCheckpoint = vectorToCurrentCheckpoint.GetAngleInDegrees();
        const float absoluteDegreesDiffToCheckpoint = AbsoluteDegreesDiff(
            m_CurrentState.degrees,
            degreesToCheckpoint
        );
        return absoluteDegreesDiffToCheckpoint;
    }

    float GetAbsoluteDegreesTowardsNextCheckpoint()
    {
        const Vec2 vectorToNextCheckpoint = GetNextCheckpointPosition() - m_CurrentState.position;
        const float degreesToCheckpoint = vectorToNextCheckpoint.GetAngleInDegrees();
        const float absoluteDegreesDiffToCheckpoint = AbsoluteDegreesDiff(
            m_CurrentState.degrees,
            degreesToCheckpoint
        );
        return absoluteDegreesDiffToCheckpoint;
    }

    void SetAllyPod(const std::shared_ptr<Pod>& allyPod)
    {
        m_AllyPod = allyPod;
    }
    void SetEnemyPod1(const std::shared_ptr<Pod>& enemyPod1)
    {
        m_EnemyPod1 = enemyPod1;
    }
    void SetEnemyPod2(const std::shared_ptr<Pod>& enemyPod1)
    {
        m_EnemyPod1 = enemyPod1;
    }

    bool IsAboutToCollideWithEnemyAndNoAlly() const
    {
        const auto ally = m_AllyPod.lock();
        if (ally && IsGoingToCollideWith(*ally))
        {
            return false;
        }
        const auto enemy1 = m_EnemyPod1.lock();
        if (enemy1 && IsGoingToCollideWith(*enemy1))
        {
            return true;
        }
        const auto enemy2 = m_EnemyPod2.lock();
        return enemy2 && IsGoingToCollideWith(*enemy2);
    }

    bool IsGoingToCollideWith(const Pod& other) const
    {
        const Vec2 currentPos = m_CurrentState.position;
        const Vec2 nextPos = currentPos + m_CurrentState.velocity * FRICTION;

        const Vec2 otherCurrentPos = other.m_CurrentState.position;
        const Vec2 otherNextPos = otherCurrentPos + other.m_CurrentState.velocity * FRICTION;

        return nextPos.DistanceTo(otherNextPos) <= POD_SIZE * 2;
    }

    //Do a rough aproximation regarding how beneficial is a collision
    float Heuristic_CollisionNicenessScore(const Pod& other) const
    {
        if (!IsGoingToCollideWith(other))
        {
            return 0;
        }

        const float baseDistance = GetDistanceToCurrentCheckpoint();

        const Vec2 currentPos = m_CurrentState.position;
        const Vec2 heuristicNextPos = currentPos + m_CurrentState.velocity * FRICTION + other.m_CurrentState.velocity * FRICTION * 10;

        const Vec2 currentCheckpoint = GetCurrentCheckpointPosition();
        const float newDistance = currentCheckpoint.DistanceTo(heuristicNextPos);

        const float result = baseDistance - newDistance;
        std::cerr << "[" << m_Tag << "] might collide with [" << other.m_Tag << "] " << result << std::endl;
        //If the newDistance is smaller then our baseDistance, maybe the collision is actually useful
        return result;
    }

    bool ShouldEnableShieldForBeneficialCollisions() const
    {
        std::cerr << "[" << m_Tag << "] :" << " checking for collisions...\n";
        static constexpr float BENEFIT_THRESHOLD = 10.f;
        const auto ally = m_AllyPod.lock();
        if (ally)
        {
            //check if you can give a "boost" to an ally
            const float scoreReportedByAlly = ally->Heuristic_CollisionNicenessScore(*this);
            if (m_DoShieldToBoostAlly && scoreReportedByAlly > BENEFIT_THRESHOLD)
            {
                return true;
            }
            else if (scoreReportedByAlly < -BENEFIT_THRESHOLD)
            {
                //Don't supress the ally
                return false;
            }

            const float scoreReportedByThis = Heuristic_CollisionNicenessScore(*ally);
            if (scoreReportedByAlly > BENEFIT_THRESHOLD)
            {
                //We are receiving a nice "boost" from an ally, keep the shield off
                return false;
            }
        }

        const auto enemy1 = m_EnemyPod1.lock();
        if (enemy1)
        {
            //check if you can really annoy an enemy
            const float scoreReportedByEnemy = enemy1->Heuristic_CollisionNicenessScore(*this);
            if (scoreReportedByEnemy < -BENEFIT_THRESHOLD)
            {
                return true;
            }
            //check if you can benefit from the enemy
            const float scoreReportedByThis = Heuristic_CollisionNicenessScore(*enemy1);
            if (scoreReportedByThis > BENEFIT_THRESHOLD)
            {
                //keep the shield off, there is good chance you get "boosted"
                return false;
            }
            if (scoreReportedByThis < -BENEFIT_THRESHOLD)
            {
                //turn on the shield, you might get yeeted
                return true;
            }
        }

        const auto enemy2 = m_EnemyPod1.lock();
        if (enemy2)
        {
            //check if you can really annoy an enemy
            const float scoreReportedByEnemy = enemy2->Heuristic_CollisionNicenessScore(*this);
            if (scoreReportedByEnemy < -BENEFIT_THRESHOLD)
            {
                return true;
            }
            //check if you can benefit from the enemy
            const float scoreReportedByThis = Heuristic_CollisionNicenessScore(*enemy2);
            if (scoreReportedByThis > BENEFIT_THRESHOLD)
            {
                //keep the shield off, there is good chance you get "boosted"
                return false;
            }
            if (scoreReportedByThis < -BENEFIT_THRESHOLD)
            {
                //turn on the shield, you might get yeeted
                return true;
            }
        }

        //nothing interesting happening, might not be worth turning on the shield
        return false;
    }

    bool IsGoingToEnterCheckpointSoon() const
    {
        const Vec2 currentCheckpoint = GetCurrentCheckpointPosition();
        Vec2 velocity = m_CurrentState.velocity;
        Vec2 aproxPositionInNextFrames = m_CurrentState.position;
        for (int i = 0; i < 6; ++i)
        {
            velocity = velocity * FRICTION;
            aproxPositionInNextFrames = aproxPositionInNextFrames + velocity;
            if (aproxPositionInNextFrames.IsInsideCircle(currentCheckpoint.x, currentCheckpoint.y, CHECKPOINT_RADIUS))
            {
                return true;
            }
        }
        return false;
    }

protected:
    PodState m_CurrentState;
    bool m_IsBoostAvailable = true;
    bool m_DoBoostOnFirstFrame = false;
    int m_NumberOfFramesToWaitBeforeBoost = 30;
    bool m_DoShieldToBoostAlly = false;
    std::shared_ptr<std::vector<Vec2>> m_SharedRaceCoordinates;
    const char* m_Tag = "";
    std::weak_ptr<Pod> m_AllyPod;
    std::weak_ptr<Pod> m_EnemyPod1;
    std::weak_ptr<Pod> m_EnemyPod2;
    int m_NumberOfTurnsSinceLastShield = -1;
    float m_LongestStretchCandidate = 0.0f;
    int m_CheckpointCount = 0;
    int m_LastCheckpointId = 0;

    static float AbsoluteDegreesDiff(const float a, const float b)
    {
        float d = b - a;
        //std::cerr << "a = " << a << " b = " << b << " d = " << d << std::endl;
        if (d < 0)
        {
            d += 360;
        }
        if (d > 180)
        {
            return 360 - d;
        }
        return d;
    }
};

int findIndexInArray(const std::vector<Vec2>& v, const Vec2& p) {
    for (int i = 0; i < v.size(); i++) {
        if (v[i] == p) {
            return i;
        }
    }
    return -1;
}

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
int main()
{
    //Initialization input
    int laps;
    std::cin >> laps;
    int checkpointCount;
    std::cin >> checkpointCount;
    std::shared_ptr<std::vector<Vec2>> checkpoints = std::make_shared<std::vector<Vec2>>(checkpointCount);
    for (int i = 0; i < checkpointCount; ++i)
    {
        std::cin >> (*checkpoints.get())[i];
    }

    auto allyPod1 = std::make_shared<Pod>(checkpoints, "Pod_1", true);
    auto allyPod2 = std::make_shared<Pod>(checkpoints, "Pod_2", false, true);
    auto enemyPod1 = std::make_shared<Pod>(checkpoints, "Bad_1");
    auto enemyPod2 = std::make_shared<Pod>(checkpoints, "Bad_2");

    allyPod1->SetAllyPod(allyPod2);
    allyPod1->SetEnemyPod1(enemyPod1);
    allyPod1->SetEnemyPod2(enemyPod2);

    allyPod2->SetAllyPod(allyPod1);
    allyPod2->SetEnemyPod1(enemyPod1);
    allyPod2->SetEnemyPod2(enemyPod2);

    // game loop
    while (1)
    {
        //Input for one game turn
        allyPod1->ReadNewStateFromCin();
        allyPod2->ReadNewStateFromCin();
        enemyPod1->ReadNewStateFromCin();
        enemyPod2->ReadNewStateFromCin();

        //Let's write some debug stuff here
        allyPod1->DebugPrintCurrentState();
        allyPod2->DebugPrintCurrentState();
        enemyPod1->DebugPrintCurrentState();
        enemyPod1->DebugPrintCurrentState();

        allyPod1->WriteCommandToCout();
        allyPod2->WriteCommandToCout();
    }
}