import { createHelia } from "helia";
import { unixfs } from "@helia/unixfs";
import { createOrbitDB } from "@orbitdb/core";

async function main() {
  const ipfs = await createHelia();
  const fs = unixfs(ipfs);
  const orbitdb = await createOrbitDB({ ipfs });

  // Create or open a log store
  const db = await orbitdb.open("lidar-data-log", { type: "eventlog" });
  await db.load();

  // Example LiDAR data (replace with actual JSON/binary data from ROS bridge)
  const lidarData = {
    timestamp: Date.now(),
    points: [
      [0.1, 0.2, 1.0],
      [0.5, 0.6, 1.1],
    ], // Simplified
  };

  await db.add(lidarData);
  console.log("LiDAR data added to OrbitDB.");

  // Read all entries
  const entries = db.iterator({ limit: -1 }).collect();
  entries.forEach((e) => console.log(e.payload.value));
}

main().catch(console.error);
