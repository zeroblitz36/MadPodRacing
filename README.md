# MadPodRacing

Here is my solution for the MadPocRacing challenge found at
https://www.codingame.com/ide/puzzle/coders-strike-back

I solved this challenge by using a combination of heuristic functions that seem to perform well enough to reach very high in the Gold League.

I attempted to keep tweaking the program to reach the Legend League, but couldn't find the sweet spot.

## Applied strategies

### OOP Love
Each one of the Pods is stored inside of a Pod object which helps me organize the code a bit.
It very useful to have when starting to do collision checks between just about all other Pods.

There is also the Vec2 struct, that has just enough functinality for it to be useful in this exercise. I would have used std::complex, but I couldn't find on https://en.cppreference.com/w/cpp/numeric/complex any way to reliably access references of the internal ```real``` and ```imag``` members, so I just gave up and decided to make my own mini struct. 

### Compensation of unwanted drifting

The pods will attempt to compensate unwanted lateral movements (drifting).
A very useful function that is being used here is ```closestPointToLine```, which helps me find out how much is the Pod is deviating laterally. Once I found out how much it is deviating, I tell the Pod to "mirror" the respective deviation, in attempt to cancel it.
This strategy was especially useful going through the Bronze and Silver Leagues.

```C++
else if (distanceTravelledOnPreviousFrame > 50 &&
    absoluteDegrees < 70.f &&
    futurePos.DistanceTo(currentCheckpoint) < currentPos.DistanceTo(currentCheckpoint))
{
    Vec2 aux = closestPointToLine(currentPos, currentCheckpoint, futurePos);
    Vec2 aux2 = aux + (aux - futurePos);
    outputPos = aux2;
}
```

 ### Going for the Home Run
 I saw that DarthBoss tries to drift on the very last checkpoint even though that really isn't necessary, as it will finish the race anyway.
 So in order to capitalize on this little weakness, I count up how many checkpoints have been triggered so far, and if it is on the last one, I tell the Pod to no longer try to do intentional drifts. Thus saving the very last units of precious distance.

### Abuse of intentional drifting

One of the key behaviors that I kept seeing DarthBoss do was how it was able to "drift" through the checkpoints in order to already start aiming for next one.
Thus I tried to mimick it the best I could.

The ```IsGoingToEnterCheckpointSoon``` method tries to predict if one of the next frame for the Pod will be inside the currently wanted checkpoint.

```C++
if (!isGoingForTheHomeRun && IsGoingToEnterCheckpointSoon())
{
    outputPos = GetNextCheckpointPosition();
    absoluteDegrees = GetAbsoluteDegreesTowardsNextCheckpoint();
}
```

This intentional drifting heuristic has priority over the "Compensation of unwanted drifting"

### Anti-spinning heuristic

One of the pains of going through the first few leagues was trying to make the Pod not spin around the checkpoint with its thrusters set to max.
Thus I came up with a little heuristic that seem to function well enough.

```C++
int thrust = 0;
if (absoluteDegrees < 90)
{
    thrust = (int)(std::ceil(100.f * std::cos(absoluteDegrees / 180.f)));
}
```

If the Pod is very well "aimed" towards the checkpoint, it will have max thrust. If the offset is getting bigger, the thrust is getting smaller to compensate, thus stopping the spinning behavior.

What is rather strange about this snippet of code, is that it is not involving the PI constant at all.
Any attempt of trying to make the snippet more "correct" by using PI, only seemed to make it worse in the end. So I left it as it is. Maybe I was just lucky and hit some magic ratio.

### Finding the right time to BOOST

I saw that one of the enemy Pods always Boosts on the 1st or 2nd frame, so I decided to mimic the behavior. My first Pod will also Boost on the first frame.

The second Pod is more conservative, it will try to wait for a sweet spot where to boost instead.
Having my Pods distance a bit from each other might decrease the chance of them colliding.

```C++
if (m_IsBoostAvailable)
{
    m_NumberOfFramesToWaitBeforeBoost--;
    if (m_DoBoostOnFirstFrame ||
        (m_NumberOfFramesToWaitBeforeBoost < 0 && absoluteDegrees < 5.f && GetDistanceToCurrentCheckpoint() > m_LongestStretchCandidate))
    {
        m_IsBoostAvailable = false;
    }
}
```

I picked a value of 30 for ```m_NumberOfFramesToWaitBeforeBoost```, and it seems to be working pretty good. 
The ```m_LongestStretchCandidate``` value is calculated at Pod construction. It finds the biggest distance between two consecutive distances, and then it multiplies it with 0.70, so that it is not too conservative with the Boost. Any decent long stretch is nice to BOOST.

I tried coming up with a more complicated heuristic, where it searches if there is any potential obstacle way in front of the Pod, but it didn't seem to improve the score.

### Bring up the SHIELD for beneficial situations

This part is the most "heuristic" of all. I saw that I managed it to make it work some bad math, and couldn't find better tweaks to it.

First of all, I created a method that tries to calculate how beneficial a collision with another Pod would be.

```C++
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
    //If the newDistance is smaller then our baseDistance, maybe the collision is actually useful
    return result;
}
```

This method is then called by ```ShouldEnableShieldForBeneficialCollisions```, that does several combinations of scores, between itself and the other Pods, in order to try to find one of these situations where the score threshold is big enough:
 1) Is this collision going to help an ally get closer to a checkpoint? return TRUE
 2) Is this collision going to suppress an ally? return FALSE
 3) Is this collision going to help an enemy? return TRUE
 4) Is this collision going to suppress an enemy? return FALSE

 Also, I decided to not let the Pod use the SHIELD sooner than 3 frames from the last SHIELD action. The acceleration cooldown of 3 frames is already painful enough as it is.

 ## Notes about code cleanliness
 The respective code is mostly definitely not something to be "submitted into production".
 Coding styles are all over the place. 
 Naming conventions are not consistent. 
 The decision to use std::shared_ptr and std::weak_ptr doesn't really give it that many cool points.
 Some magic numbers are stored in nice constexpr values, some are just thrown in the code with no name to help decipher what it means.
 The code that I've written is here is just something that I kept iterating in order to get higher and higher scores. But it definitely can be cleaned up.

## What can be done better?
From all the tweaks and heuristics functions that I tried to make, the best that I reached was Gold League 3rd place with a score of 40.78. The DarthBoss has a score of 41.79, so I am really close :).

The ```Heuristic_CollisionNicenessScore``` can definitely be done better, but I didn't manage to wrap my head around the formulas for Elastic Collisions https://en.wikipedia.org/wiki/Elastic_collision

I strongly believe that making more heuristics and tweaks would take way more time, and the real solution would be to start dwelving into some Machine Learning.
The guys and gals that presented the Gold League version of MadPodRacing were nice enough to also show all the numerical constants and details necessary to simulate your very own MadPodRacing arena. 

Thus it is very possible to train your ultimate Pod duo offline, and then test out a hardcoded copy of the best neural network that was cooked up by your PC. The enemy of the AI-in-training could also be this heuristic approach, as it seems to be working pretty well.