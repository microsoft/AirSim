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

function verifyHMAC(req, res, next) {
  const signature = req.headers['x-proxy-signature'];
  const timestamp = req.headers['x-proxy-timestamp'];
  const body = JSON.stringify(req.body);
  const message = `${req.method}:${req.path}:${timestamp}:${body}`;
  const computed = crypto.createHmac('sha256', proxySecret).update(message).digest('hex');

  if (signature !== computed) return res.status(401).json({ error: 'Invalid HMAC' });
  next();
}

app.post('/sign-message', verifyHMAC, async (req, res) => {
  const { message, require2FA, description, metadata } = req.body;

  if (require2FA) {
    const opId = crypto.randomUUID();
    const timestamp = Date.now().toString();
    const approvalReqBody = { operationId: opId, description, metadata };
    const hmac = crypto.createHmac('sha256', proxySecret).update(`POST:/request-approval:${timestamp}:${JSON.stringify(approvalReqBody)}`).digest('hex');

    await fetch(`${TWOFA_GATEWAY_URL}/request-approval`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'x-proxy-signature': hmac, 'x-proxy-timestamp': timestamp },
      body: JSON.stringify(approvalReqBody)
    });

    // Polling for approval
    const start = Date.now();
    let approved = null;
    while (Date.now() - start < 300000) {
      const statusRes = await fetch(`${TWOFA_GATEWAY_URL}/approval-status/${opId}`);
      const status = await statusRes.json();
      if (status.approved !== null) { approved = status.approved; break; }
      await new Promise(r => setTimeout(r, 2000));
    }

    if (!approved) return res.status(403).json({ error: 'Rejected or Timeout' });
  }

  const signature = await wallet.signMessage(message);
  res.json({ signature, address: wallet.address });
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`🔑 Keyring Proxy on ${PORT}`));
