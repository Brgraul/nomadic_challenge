
Prioritization of subchallenges
Possible problems include:
Vegetation management: Fallen trees or branches touching a line, or threatening to fall onto it.
DLR: Damaged wires or connections, which may break or become unusually hot.
Sag: Lines hanging too low, bringing them dangerously close to trees or the ground.
Galloping (sin wave): Strong wind causing wires to swing, potentially touching each other or damaging equipment.
Ice: Ice building up on wires, adding weight and pulling them down.
Fire: Fire near a power line, threatening equipment and making the area unsafe.
Choose one problem, or a small combination. Decide what you would measure and how those measurements would help an operator act.

Anomaly detection based on sensor data. Not necessarily vision
Drones are controlled separately
How to factor in incoming reports for routing (Agentic processing of incoming reports?)
Jev for confidence.

Questions for Andreas
How often do these challenges appear? Which one is more costly/utility companies care more about?
Is there any value in pursuing challenges you already tackle?
What’s more important, the specific subproblem, or how we optimize navigation?
Should we focus on the perecentage of blackouts caused by the specific cause
What should be skeptic about?
Initial grid data is true
Report info is to be validated

Edge cases:
Power outage when you’re charging, you loose a drone
Flight risk
Allowed flight areas

Speed,Tree height, Sag height clearance, 

Are we focussing on problems that are happened or the one possibility of them?



Are there ways of detecting a live  

Notes for us
Atm, the drones can’t take actions
However, sounds like there’s room for an LLM layer to recommend human inspections or human actions



Questions to address by us
What properties of the cable are we measuring?
How many sampling points for a single line?
What kind of sensor we use?




Notes Interview

blackout, where should operator look first
—> detect cause of blackout

when does evidence justify a crew
consider real world chaos, clearify assumptions


focus on galloping and vegetation
current vibration pattern will change, anomalies. not actual physical signs, 
false positives are the worst, confidence is key for decision, what the best candidate to spend your time

what aman aks and his answer we can image llm power

including risk on drone charging if the lines go dead
assume that shortcuts are possible
assume that people are unreliable
pattern recognotion of people can be considered


Timeline & team split

13:00 - We must have agreed on what signals we measure and sampling strategy (Kerem)
		Kerem: Find a method for detecting line material changes (conductivity, discontinuity, or resistance) that does not require power flowing through the line.
	We must have a system diagram and a workflow diagram and spec (All)
14:00 - We must have agreed on where are we initializing the drones (All)
15:00 - First working version of the routing algorithm, even if sensors are mocked (R+A)
	First outline of the pitch must be there (Liam)
16:00 - We must have an end to end prototype, with real data

Liam
General problem framing
Identify data gaps
Outline for pitch / presentation
Execute on the demo

Kerem
Pls look into dynamic line rating
Sampling strategy for the sensing
How many measurements
What do we measure

Ayush + Raúl 


Requirements & Spec

We’re building a multi-agent system, that forms a hypothesis about the root cause, and then can use the 6 drones to collect data. Our system must consider these constraints:
Drones 30min autonomy, 3h charging, charges only on energized lines (Routing algo constraints)
Drones should effectively collect data to proove or disprove hypothesis
Based on drone information, orchestrator can dynamically reroute them mid-flight

Orchestrator has access to the following tools:
calculate_pf: Based on existing information calculates powerflows for the network
optimize_routes: based on the tasks deployed to the drones  
send_sensor: Sends sensors to specific map coordinates
communicate_operators: on hypothesis update, probabilitically evaluates the best remedial actions the grid operator could take

Drones have the following tools:
measure_ @kem



What should the system able to do
It should have an agentic layer 
Each drone could have a shared state log of the grid
Task allocation and distributed path planning by the Master Agent
It should tell where to initialize the 6 drones
It should have some route optimization algorithm (Use libraries & pre-made code where possible)
It should suggest operators actions at a fixed interval or when specific events happen (Discovery of anomalies) -> LLM driven
For recommended actions, it should do some kind of confidence intervals (JEV-like decision model, OpenAI decisions API)










DEMO TO BE UPDATED
abstarct:
demo a storm that is approaching, combined with wether data
highlight potentially affected lights of the grid (vegeational cause)
system ranked by damange risks and optimzied after efficeny how a drone can reach them (fastest way+most data collected+charging possibility)
parameters:
 prevent false positives at all costs—>show sth seems like a true positive, but after we measure it turns out to be a false positive.
consider the beeing everywhere without physically being everywhere approach
narrow done on one drone route and decision mechanism
the system comes up with hypothesis on whats the cause:
 H1 H2 H3, display supporting evidence
see data input and what hypothesis gains or loses support
demonstarte a false positive “All signs are suggesting this, but the last data parameter show that its normal”
revise hypothesis confidence, reprioirtize locations and ways of drones to get there
Goal: not just identify suspected faults. Forming Hypothesis, tests them with drone measurements, learns form evidence, recommend what next, Considering risk, probability, Data missing and there, length of way and charging possibility
Claude sum up:
Hypotheses with visible confidence shifts. This matches what Andreas stressed: confidence decides action, and false positives are the worst outcome. This is your winning element.
The false-positive moment. Present it as “the system caught it before a crew was sent.” That shows you value avoiding false positives without breaking any rule. Put a number on it, e.g. “saved one crew dispatch ≈ X hours.” The idea “prevent false positives at all costs” fits with demoing one, because the demo shows the prevention.
Charging as a real constraint. A drone can only charge on a live line, and after a storm many lines are dead. Your edge case of losing a drone while it charges, because its line dies, is a great moment. Put it in the demo.
An LLM layer for the reports. Use it to turn messy reports into structured claims (location, symptom, time, how reliable the reporter is). The founder said people are unreliable, so give each report a reliability weight.
Drop “storm approaching.” The challenge starts after the storm has passed, so an approaching storm is a different problem. Use the storm’s wind/gust data only as a prior: a forest corridor that took high gusts gets a higher starting risk. Animating a storm would also eat hours.
Don’t lead with vision. The founder said the solution should be anomaly detection from sensor data, not necessarily cameras. For vegetation, a perched drone could measure:
Also say what the sensors can’t do: they can’t pinpoint where along a span the problem is, can’t predict whether a leaning tree will fall, and have weak electrical signals on a dead line. Being honest about limits is explicitly part of the deliverable.
 - Current / magnetic field (from the charging coil): is the line live, and did load change?
 - Vibration spectrum (accelerometer): a branch resting on a conductor changes how the wire vibrates. This is the “vibration pattern will change” point from the interview.
 - Conductor angle (inclinometer): extra sag from weight on the line.
 - Partial discharge / arcing (acoustic or RF): contact with vegetation on a live line.
 - Camera: confirmation only, as the last step before a crew is dispatched.
 3. Don’t build a real multi-objective route optimizer. Drones are flown by a separate system, so your value is deciding where to go next, not path planning. Use a simple score:priority = risk × expected information gain ÷ flight time
 Only include targets reachable within 30 minutes and close to a live line where the drone can land. Andreas said to assume shortcuts are possible, so straight-line distance ÷ speed is enough.
 4. Show all five drones first, then zoom in on one. Placing all five with line/node IDs is a required deliverable. After that, a deep dive on one drone is fine.
 5. Make H3 a non-vegetation hypothesis. If H1–H3 are all “tree on segment X/Y/Z,” the false positive has nowhere to go. Better:
Then the false positive can resolve into H3 or H2.
 - H1: tree contact on segment A
 - H2: tree contact on segment B (the report location is ambiguous)
 - H3: not vegetation (a protection trip, or wires clashing in the wind)
 6. Don’t let the LLM produce the confidence numbers. Judges will distrust that. The LLM extracts facts; a simple Bayesian update (prior × likelihood ratio for each piece of evidence) computes confidence. “Jev” in your notes probably means something like expected value of information, which fits this exactly. Confirm with your team if unsure.
 7. Count “no evidence” as evidence. The brief asks for it explicitly. A drone on the adjacent span reading a normal vibration spectrum should lower H1’s confidence, not leave it unchanged.
Suggested demo flow (~5 min)
t=0: Six reports come in. The LLM structures them and assigns reliability weights. Combined with which lines are dead, forest cover and gust data, the system ranks candidate segments.
Placement: Five drones are placed on live lines next to the top candidates, with the IDs shown.
Drone 1 measures. H1 rises from 0.55 to 0.78. Everything points to a tree on line A: forest corridor, peak gusts, two “tree on line” reports.
The false positive: a second, cheap reading (vibration and angle normal on that span) drops H1 to 0.25. The reports had described a tree beside the road, not on the line. No crew is sent.
Reprioritize: drone 2 is redirected to segment B. On the way, its planned charging line goes dead, so it re-plans to land on a live neighboring line.
Recommendation card: “Dispatch crew to segment B. Confidence 0.87. Evidence: … Still missing: … Next measurement: …”
Make the action thresholds explicit on screen, for example:
≥ 0.8 with at least 2 independent types of evidence: send a crew
0.4 – 0.8: measure more
< 0.2: deprioritize
Wow factor: add a timeline slider plus a toggle like “what if this reading had been abnormal?” so judges can change evidence themselves and watch confidence and routes update. Everything should be clearly labelled as hypothetical, as the brief requires.



What the package gives us
Lines: 64, with direction, length, voltage, operator and each line’s OpenStreetMap ID
Nodes: 59, 15 of them substations
Reports: 6 vague ones, with rough areas
Storm: one rough outline
Known line state: only one, LINE_024 is switched off
Gaps, how we fill them, and what it does to suspicion
Gap
Data we combine
Effect on suspicion
Can a tree hit the line? (strongest signal)
Line position + OpenStreetMap forest + tree height (Meta 1 m canopy map or Bavarian laser scans)
A tree taller than its distance to the wire → up a lot. No such trees → down a lot
Which way do trees fall?
Line direction + assumed wind direction + which side the forest is on
Forest upwind of a line crossing the wind → up. Forest downwind → down
How fragile are the trees?
German tree species map + soil moisture (DWD)
Spruce (shallow roots) or soaked ground → up. Leafless deciduous trees in late autumn → slightly down
Where are the towers?
OpenStreetMap, using the line IDs in the package
Not a probability effect: tells you where to place drones and how many sampling points each line has
How much is each report worth?
Report areas + line map
Area covers 13 lines (R05) → little effect per line. Covers 4 lines (R02) → more. LINE_026 is in two reports (vegetation + protection trip) → biggest boost, and our best false-positive setup
Is there an innocent explanation?
Report details + outside facts (switching logs, lightning data, generator test log, fire brigade calls, road closures ★)
R03 explained by the planned LINE_024 switch-off → tree down. R04 flash explained by the generator test → down
Which lines are dead? ★
Line topology + devices going offline (solar systems, EV chargers, smart meters)
Line inside the outage area → up. Also shows where drones can’t charge
What’s normal for each line? ★ (Nomadic’s advantage)
Drone reading after the storm + the same line’s readings from before it
Unusual vibration → up. Matches normal → down, even when every other sign pointed to a tree. This is our false-positive moment
Who is affected?
Substations + population map + renewable plant register
Changes priority, not probability: more people or a big solar/wind park → higher in the queue







Check impact of currents
Vegetative data for risk calculation, where to send the drones
priority = (datasets google maps)risk × expected information gain ÷ flight time x Impact (households and powerplants)









































































KEREM:
	4 wire resistance
In 10km distance :	
 TDR and 4 wire resistance
In 200m-500m 
	Optics : LiDAR + Camera = exact scene detection

	


ALGORITHMS currently in practice for the same scenario:
Ant colonization Algorithm 
GEMINI SUMUP: Coordinates Area Coverage (Search): It routes multiple UAVs across a grid map so they explore unsearched zones efficiently. The "repulsive pheromones" actively force drones away from regions other drones have already scanned, eliminating redundant paths and maximizing search coverage. 

Dynamic Rerouting (Adaptation): As the wildfire spreads (driven by wind and weather models) or obstacles appear, the algorithm instantly updates the grid weights and recalculates flight paths on-the-fly to reinforce containment lines where suppression is lagging.



