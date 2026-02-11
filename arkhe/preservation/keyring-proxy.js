// keyring-proxy/src/index.js - Arkhe(n) Keyring Proxy
import express from 'express';
import { Wallet } from 'ethers';
import crypto from 'crypto';
import dotenv from 'dotenv';
import fetch from 'node-fetch';

dotenv.config();

const app = express();
app.use(express.json());

const privateKey = process.env.AGENT_PRIVATE_KEY;
const wallet = new Wallet(privateKey);
const proxySecret = process.env.PROXY_HMAC_SECRET;
const TWOFA_GATEWAY_URL = process.env.TWOFA_GATEWAY_URL || 'http://2fa-gateway:4000';

function generateHMAC(method, path, timestamp, body) {
  const message = `${method}:${path}:${timestamp}:${JSON.stringify(body)}`;
  return crypto.createHmac('sha256', proxySecret).update(message).digest('hex');
}

function verifyHMAC(req, res, next) {
  const signature = req.headers['x-proxy-signature'];
  const timestamp = req.headers['x-proxy-timestamp'];
  if (!signature || !timestamp) return res.status(401).json({ error: 'Missing HMAC headers' });
  const computed = generateHMAC(req.method, req.path, timestamp, req.body);
  if (signature !== computed) return res.status(401).json({ error: 'Invalid HMAC' });
  next();
}

async function requestApproval(description) {
  const operationId = crypto.randomUUID();
  const timestamp = Date.now().toString();
  const body = { operationId, description };
  const hmac = generateHMAC('POST', '/request-approval', timestamp, body);

  await fetch(`${TWOFA_GATEWAY_URL}/request-approval`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'x-proxy-signature': hmac,
      'x-proxy-timestamp': timestamp
    },
    body: JSON.stringify(body)
  });

  const start = Date.now();
  while (Date.now() - start < 5 * 60 * 1000) {
    const statusRes = await fetch(`${TWOFA_GATEWAY_URL}/approval-status/${operationId}`);
    if (statusRes.ok) {
      const data = await statusRes.json();
      if (data.approved !== null) return data.approved;
    }
    await new Promise(r => setTimeout(r, 2000));
  }
  return false;
}

app.post('/sign-message', verifyHMAC, async (req, res) => {
  const { message, requireApproval, approvalDescription } = req.body;
  if (requireApproval) {
    const approved = await requestApproval(approvalDescription || `Sign: ${message.substring(0, 30)}`);
    if (!approved) return res.status(403).json({ error: 'Rejected by owner' });
  }
  const signature = await wallet.signMessage(message);
  res.json({ signature, address: wallet.address });
});

app.get('/address', (req, res) => res.json({ address: wallet.address }));

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`🔑 Keyring Proxy v3.0 ativo na porta ${PORT}`));
