import { setupOrbitDB } from "./orbitdb/setupOrbitDB.js";
import { addLidarData, queryAllEntries } from "./orbitdb/lidarData.js";

async function main() {
  const db = await setupOrbitDB();

  // Example: Add dummy lidar data
  await addLidarData(db, { x: 1, y: 2, z: 3 });

  // Query and print all entries
  const entries = queryAllEntries(db);
  entries.forEach((entry) => {
    console.log(entry.payload);
  });
}

main().catch(console.error);
