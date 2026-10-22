# Simulation of Autonomous Drone Swarm Communication Using Computer Networks

A desktop application that simulates a swarm of autonomous drones and the **wireless
ad hoc network** that connects them.  Drones move autonomously, exchange JSON messages
over **real UDP sockets**, route those messages through intermediate drones with
**BFS multi-hop forwarding**, and the network can be broken on purpose so that the
effect on performance can be measured.

Everything the report describes is implemented and every number in the report comes
from an actual simulation run.

---

## 1. Quick start

```bash
pip install -r requirements.txt

python main.py             # start the interactive dashboard
python run_experiments.py  # run all 5 experiments head-less, write CSV + charts
python -m pytest tests -q  # run the automated tests
```

No installation of a GUI toolkit is required: the dashboard is built with **tkinter**,
which is part of the Python standard library.

---

## 2. What the application does

| Area | Implementation |
|---|---|
| Drone movement | Random waypoint model, world boundaries, obstacle avoidance (`drone.py`) |
| Network topology | Graph rebuilt every tick from the current positions (`topology.py`) |
| Message transport | One real UDP socket per drone on `127.0.0.1`, JSON payloads (`comms.py`) |
| Routing | Breadth-first search for the fewest-hop path, multi-hop forwarding (`topology.py`, `simulation.py`) |
| Fault injection | Disconnect / reconnect a drone, disable / restore a link, packet loss, link delay (`failures.py`) |
| Performance metrics | PDR, loss, latency (avg/min/max/p95), throughput, connectivity, recovery time (`metrics.py`) |
| Experiments | 5 experiment groups, CSV export, matplotlib charts (`experiments.py`) |
| User interface | Dark themed monitoring dashboard with canvas, controls, log and charts (`ui/`) |

---

## 3. Using the dashboard

```
+---------------------------------------------------------------+
| Start | Reset | Step | Route to D1 |  duration | Export | Run  |
+---------+---------------------------------------+-----------+
| control |                                       | Dashboard |
| panel   |        simulation canvas              | Log       |
|         |                                       | Experiments|
|         |                                       | Charts    |
+---------+---------------------------------------+-----------+
| status bar                                                   |
+---------------------------------------------------------------+
```

* **Canvas** – drones, wireless links, disabled links (red dashed), obstacles and the
  selected route.  Click a drone to inspect it.
* **Swarm configuration** – number of drones, communication range, drone speed,
  simulation speed, message interval.
* **Network conditions** – simulated packet loss (0–50 %) and artificial link delay.
* **Failure simulation** – disconnect / reconnect a drone and disable / restore one of
  its links.  The dashboard immediately shows whether the destination is still
  reachable and which alternative route is used.
* **Dashboard** – active drones, active links, connected components, isolated drones,
  transmitted / delivered / dropped / undeliverable messages, PDR, loss, latency and
  throughput, plus the reachability of the selected drone.
* **Communication log** – every event with time, source, destination, message type and
  status (`DELIVERED`, `FORWARDED`, `DROPPED`, `UNREACHABLE`, `DISCONNECT`, …).
* **Experiments** – run the whole experiment plan from the GUI, compare the results in
  a table and export them as CSV.
* **Charts** – live PDR and latency graphs of the running simulation.

---

## 4. Code structure (division of work)

```
CN Project/
├── main.py                     # launch the dashboard
├── run_experiments.py          # head-less experiment runner (CLI)
├── requirements.txt
├── drone_swarm/
│   ├── config.py               # all tunable parameters
│   ├── drone.py                # (Student 1) drone agents, movement, obstacles
│   ├── comms.py                # (Student 2) UDP sockets, JSON messages, validation
│   ├── topology.py             # (Student 3) graph, neighbour detection, BFS routing
│   ├── failures.py             # (Student 3) fault injection and recovery tracking
│   ├── metrics.py              # (Student 4) performance indicators
│   ├── simulation.py           # engine that connects every module
│   ├── experiments.py          # (Student 4) experiment plan, CSV export, charts
│   └── ui/
│       ├── app.py              # main window, control panel, dashboard, log
│       └── canvas.py           # rendering of the swarm and the topology
├── tests/                      # pytest suite (30 tests)
└── results/                    # CSV + charts produced by the experiments
```

---

## 5. Message format

Every datagram is one JSON object:

```json
{
  "message_id": "MSG0015",
  "sender_id": "D1",
  "receiver_id": "D5",
  "message_type": "POSITION_UPDATE",
  "sequence_number": 15,
  "created_at": 12.5,
  "payload": {"x": 120.0, "y": 250.0, "status": "ACTIVE", "speed": 42.0},
  "route": ["D1", "D2", "D4", "D5"],
  "hop": 1,
  "next_hop": "D4"
}
```

* `route` contains the full path computed by the BFS router, `hop` is the index of the
  drone that currently holds the message and `next_hop` the neighbour it is sent to.
* `net.sent_at` / `net.delay` are added by the transport layer and carry the modelled
  one-way link delay used for the latency measurement.
* Message types: `POSITION_UPDATE`, `STATUS_UPDATE`, `ALERT`.

---

## 6. Metrics

| Metric | Definition |
|---|---|
| PDR | delivered unique messages / eligible unique messages generated × 100 |
| Packet loss | (eligible messages − delivered) / eligible messages × 100, split into *simulated drops* and *unreachable* messages |
| Latency | arrival time − creation time, reported as average, min, max and 95th percentile |
| Throughput | delivered payload bytes / measurement duration |
| Connectivity | active drones, active links, connected components, isolated drones, reachability |
| Recovery time | time between the injection of a fault and the restoration of the lost connectivity |

---

## 7. Experiments

`python run_experiments.py --duration 90` runs the complete plan (each run: 10 drones,
90 simulated seconds, one message per second) and writes
`results/drone_swarm_results.csv` plus the charts.  Results of the last run in this
repository:

| Experiment | PDR % | Loss % | Unreachable % | Latency ms | Throughput B/s | Active links | Components |
|---|---|---|---|---|---|---|---|
| Exp1 Baseline (10 drones) | 98.89 | 1.11 | 1.11 | 23.63 | 54.97 | 38 | 1 |
| Exp2 Packet loss 10 % | 84.44 | 15.56 | 0.00 | 22.36 | 46.80 | 35 | 1 |
| Exp3 Drone disconnection | 100.00 | 0.00 | 0.00 | 22.33 | 55.50 | 31 | 1 |
| Exp4 Swarm size 5 | 95.56 | 4.44 | 4.44 | 22.44 | 51.76 | 9 | 1 |
| Exp4 Swarm size 10 | 100.00 | 0.00 | 0.00 | 22.84 | 55.01 | 31 | 1 |
| Exp4 Swarm size 20 | 100.00 | 0.00 | 0.00 | 23.19 | 54.78 | 140 | 1 |
| Exp5 Range 80 px | 28.89 | 71.11 | 70.00 | 30.00 | 16.26 | 8 | 4 |
| Exp5 Range 120 px | 70.00 | 30.00 | 30.00 | 33.87 | 38.59 | 16 | 2 |
| Exp5 Range 160 px | 95.56 | 4.44 | 4.44 | 30.75 | 52.69 | 28 | 1 |
| Exp5 Range 200 px | 98.89 | 1.11 | 1.11 | 23.71 | 55.33 | 32 | 1 |
| Exp5 Range 240 px | 100.00 | 0.00 | 0.00 | 22.33 | 55.01 | 44 | 1 |

Findings:

* **Packet loss** directly reduces the PDR (98.9 % → 84.4 %) and the throughput
  (55.0 → 46.8 B/s), while the latency of the surviving messages stays unchanged – a
  lost packet does not slow down the others.
* **Communication range** is the dominant factor.  At 80 px the network splits into
  4 components, 70 % of the messages have no route at all and the latency of the
  remaining multi-hop messages rises from 22 ms to 30 ms (1.5 hops instead of 1.2).
  At 240 px the swarm is always connected and the PDR reaches 100 %.
* **Swarm size** mainly increases the number of links (9 → 31 → 140) and therefore the
  number of alternative routes.  PDR and throughput stay almost constant because the
  message rate is kept constant, which makes the comparison fair.
* **A disconnected drone** (Exp3, offline from 30 s to 60 s) does **not** interrupt the
  traffic: the remaining nine drones keep the swarm connected and the router simply
  uses a different path (mean hop count 1.29).  The number of links drops from 38 to
  31 and the recovery time of the run was **29.75 s**.

---

## 8. Tests

```
tests/test_drone.py        movement, boundaries, obstacle avoidance
tests/test_topology.py     link creation, BFS routes, partitions, disabled links
tests/test_comms.py        JSON encoding, real UDP transmission, duplicate detection
tests/test_metrics.py      PDR, loss, latency statistics, throughput, failure events
tests/test_simulation.py   end-to-end delivery, multi-hop, failures, experiments
```

---

## 9. Limitations

* The drones exist only in software; there is no flight dynamics model.
* The wireless channel is modelled (range, loss, delay) and not physically emulated –
  there is no interference, contention or propagation model.
* All UDP traffic runs over the loopback interface of one computer.
* The router is a plain fewest-hop BFS, not a standardised ad hoc protocol such as
  AODV.
* The message rate is global (one message per second for the whole swarm) so that the
  swarm size experiment stays comparable.
* The results describe this simulation and cannot be transferred directly to real
  drone swarms.
