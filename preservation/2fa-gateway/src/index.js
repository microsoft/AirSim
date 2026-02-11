import express from 'express';
import TelegramBot from 'node-telegram-bot-api';
import crypto from 'crypto';
import dotenv from 'dotenv';

dotenv.config();

const app = express();
app.use(express.json());

const bot = new TelegramBot(process.env.TELEGRAM_BOT_TOKEN);
const ownerChatId = process.env.TELEGRAM_CHAT_ID;
const pendingApprovals = new Map();
const PROXY_HMAC_SECRET = process.env.PROXY_HMAC_SECRET;

function verifyProxyHMAC(req) {
  const signature = req.headers['x-proxy-signature'];
  const timestamp = req.headers['x-proxy-timestamp'];
  const body = JSON.stringify(req.body);
  const message = `${req.method}:${req.path}:${timestamp}:${body}`;
  const hmac = crypto.createHmac('sha256', PROXY_HMAC_SECRET).update(message).digest('hex');
  return crypto.timingSafeEqual(Buffer.from(signature), Buffer.from(hmac));
}

app.post('/request-approval', (req, res) => {
  if (!verifyProxyHMAC(req)) {
    return res.status(401).json({ error: 'Invalid HMAC' });
  }

  const { operationId, description, metadata } = req.body;
  const baseUrl = process.env.PUBLIC_URL || `http://localhost:${process.env.PORT || 4000}`;
  const approveUrl = `${baseUrl}/approve/${operationId}`;
  const rejectUrl = `${baseUrl}/reject/${operationId}`;

  const message = `
🔐 *Arkhe(n) SIWA Approval Required*

*Agente:* Plex Preservation v3.0
*Operação:* ${description}
*Impacto:* ${metadata.severity || 'N/A'}
*ID:* \`${operationId}\`

[✅ Aprovar](${approveUrl}) | [❌ Rejeitar](${rejectUrl})
  `;

  bot.sendMessage(ownerChatId, message, { parse_mode: 'Markdown' });

  pendingApprovals.set(operationId, {
    approved: null,
    createdAt: Date.now()
  });

  res.json({ status: 'pending', operationId });
});

app.get('/approve/:id', (req, res) => {
  const op = pendingApprovals.get(req.params.id);
  if (op) { op.approved = true; res.send('✅ Approved.'); }
  else { res.status(404).send('Not found.'); }
});

app.get('/reject/:id', (req, res) => {
  const op = pendingApprovals.get(req.params.id);
  if (op) { op.approved = false; res.send('❌ Rejected.'); }
  else { res.status(404).send('Not found.'); }
});

app.get('/approval-status/:id', (req, res) => {
  const op = pendingApprovals.get(req.params.id);
  if (!op) return res.status(404).json({ error: 'Not found' });
  if (Date.now() - op.createdAt > 300000) {
    pendingApprovals.delete(req.params.id);
    return res.json({ approved: false, reason: 'timeout' });
  }
  res.json({ approved: op.approved });
});

const PORT = process.env.PORT || 4000;
app.listen(PORT, () => console.log(`📱 2FA Gateway on ${PORT}`));
