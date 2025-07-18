import { createHelia } from 'helia'
import { createLibp2p } from 'libp2p'
import { noise } from '@chainsafe/libp2p-noise'
import { yamux } from '@chainsafe/libp2p-yamux'
import { webSockets } from '@libp2p/websockets'
import { identify } from '@libp2p/identify'
import { circuitRelayTransport } from '@libp2p/circuit-relay-v2'
import { OrbitDB } from 'orbit-db'

async function main() {
  // Create libp2p node with transports and encryption
  const libp2p = await createLibp2p({
    transports: [
      webSockets(),
      circuitRelayTransport()
    ],
    connectionEncrypters: [noise()],
    streamMuxers: [yamux()],
    services: {
      identify: identify()
    }
  })

  // Create Helia node using libp2p instance
  const helia = await createHelia({ libp2p })

  // Create OrbitDB instance
  const orbitdb = await OrbitDB.createInstance(helia)

  // Create / Open a key-value store
  const kvStore = await orbitdb.keyvalue('my_kv_store')

  // Put some data
  await kvStore.put('hello', 'world')

  // Get data
  const value = kvStore.get('hello')
  console.log('Value for "hello":', value)

  // Close the store
  await kvStore.close()

  // Stop libp2p and Helia
  await orbitdb.stop()
  await helia.stop()
  await libp2p.stop()
}

main().catch(console.error)
