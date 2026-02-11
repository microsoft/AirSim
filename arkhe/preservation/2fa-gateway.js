// gateway.js - SIWA 2FA Telegram Gateway (using grammy)
import { Bot, InlineKeyboard } from "grammy";
import express from "express";
import dotenv from "dotenv";

dotenv.config();

const bot = new Bot(process.env.TELEGRAM_BOT_TOKEN);
const app = express();
app.use(express.json());

const pendingApprovals = new Map();

// 1. Recebe a solicitação do Keyring Proxy (via rede privada)
app.post("/request-approval", async (req, res) => {
    const { opId, desc, nonce, metadata } = req.body;

    const keyboard = new InlineKeyboard()
        .text("✅ Aprovar", `approve_${opId}`)
        .text("❌ Rejeitar", `reject_${opId}`);

    await bot.api.sendMessage(process.env.TELEGRAM_CHAT_ID,
        `🤖 *SIWA: Solicitação de Assinatura*\n\n` +
        `📝 *Ação:* ${desc}\n` +
        `🔐 *Nonce:* \`${nonce}\`\n` +
        `📂 *Impacto:* ${metadata ? metadata.severity : 'N/A'} de perda detectada.\n\n` +
        `Arquiteto, deseja autorizar a restauração?`,
        { parse_mode: "Markdown", reply_markup: keyboard }
    );

    pendingApprovals.set(opId, { approved: null, createdAt: Date.now() });
    res.status(202).json({ status: "pending", opId });
});

// 2. Escuta o seu toque no Telegram
bot.callbackQuery(/^(approve|reject)_(.+)$/, async (ctx) => {
    const [action, opId] = ctx.match.slice(1);
    const op = pendingApprovals.get(opId);

    if (op) {
        op.approved = action === "approve";
        await ctx.editMessageText(action === "approve" ? "✅ *Operação Autorizada pelo Arquiteto*" : "❌ *Operação Negada*", { parse_mode: "Markdown" });
    } else {
        await ctx.answerCallbackQuery("Operação expirada ou não encontrada.");
    }
    await ctx.answerCallbackQuery();
});

// Endpoint para consulta de status (Internal polling)
app.get("/approval-status/:id", (req, res) => {
    const op = pendingApprovals.get(req.params.id);
    if (!op) return res.status(404).json({ error: "Not found" });
    if (Date.now() - op.createdAt > 5 * 60 * 1000) {
        pendingApprovals.delete(req.params.id);
        return res.json({ approved: false, reason: "timeout" });
    }
    res.json({ approved: op.approved });
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`📱 2FA Gateway (grammy) ativo na porta ${PORT}`));
bot.start();
