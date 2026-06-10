# Galileo OSNMA

OSNMA (Open Service Navigation Message Authentication) lets a receiver prove that
the Galileo I/NAV ephemeris, clock and health it decoded were really signed by
Galileo — defeating navigation-message spoofing. `gnss-rcv` decodes the OSNMA
bits from the live E1-B signal and verifies them with the
[`galileo-osnma`](https://github.com/daniestevez/galileo-osnma) crate —
automatically, on any E1-B (`--sig E1B`) run.

This note records how the pipeline is wired, the trust anchor it ships, and what
we have and haven't been able to demonstrate on real recordings.

## The trust chain

OSNMA authenticates in three nested layers, each anchored by the one below:

1. **TESLA MACs.** Every 30 s subframe carries MAC tags over the navigation data.
   The MAC key is **disclosed one subframe later** — so a tag can't be forged in
   real time, and a receiver validates it after the key arrives.
2. **DSM-KROOT.** The root of the TESLA key chain (KROOT) is broadcast with an
   **ECDSA signature**. Verifying it bootstraps trust in the whole chain.
3. **Merkle tree + DSM-PKR.** The ECDSA public key is a leaf of a Merkle tree
   whose 32-byte **root** the GSC publishes out of band. A DSM-PKR message carries
   the public key plus its Merkle proof, so a receiver holding only the root can
   authenticate the key.

The single out-of-band secret a receiver needs is therefore the **Merkle root**
(to verify a DSM-PKR) or, equivalently, a **known-good public key** (to verify the
DSM-KROOT directly, skipping the DSM-PKR).

## The receiver pipeline

| Stage | Where | What |
|---|---|---|
| Extract | [`galileo_inav.rs`](../src/galileo_inav.rs) | The 40-bit OSNMA field is the odd page's "Reserved 1" at `page[132..172]` (just after the 16-bit Data j). `InavDecoder::assemble` lifts it into `InavWord.osnma` alongside the 128 data bits. |
| Timestamp | [`navigation.rs`](../src/navigation.rs) | Each decoded page is buffered with the **GST at which it was transmitted**: the word-5 TOW anchor plus real elapsed time. |
| Feed + report | [`receiver.rs`](../src/receiver.rs) | After the parallel channel step, the receiver drains every Galileo channel's page buffer into one shared verifier and logs each SV that reaches an authenticated state — the same sequential phase as the position fix. |
| Verify | [`osnma.rs`](../src/osnma.rs) | `OsnmaVerifier` wraps `galileo_osnma::Osnma`; `pack_msb` packs our one-bit-per-byte arrays MSB-first into the crate's 16-byte I/NAV word and 5-byte OSNMA message. |

### Why per-page GST timing is forgiving

The GST is an input to the MAC, so it must be *correct* — but the crate floors
every fed GST to its 30 s subframe (`gst.gst_subframe()`) before using it. So the
per-page GST only has to land in the **right 30 s subframe**, which the word-5
anchor (`eph.tow` read straight from the message, plus sample-clock elapsed time)
achieves with sub-second margin. We don't need to reconstruct each page's exact
2 s slot.

## Trust anchors (built in, auto-selected by epoch)

Each anchor is a (Merkle root, ECDSA P-256 public key, PKID) the GSC published,
valid for a range of GST weeks. All three known epochs are built into
[`osnma.rs`](../src/osnma.rs) (`ANCHORS`), and the receiver picks the right one
from the **decoded GST week** — `OsnmaVerifier::for_gst_week(week)` — so it works on
any capture without the user choosing:

| Epoch | From | Merkle root | Public key (PKID) |
|---|---|---|---|
| **2023** | (pre-renewal) | `0E63F552…0148B8` | `0374A925…F0F6DB0` (PKID 1) |
| **2024** | 2024-01-15 (week 1273) | `832E15ED…F842F04E` | `0397EB43…3501F73B` (PKID 1) |
| **2025** | 2025-12-10 (week 1372) | `832E15ED…F842F04E` | `02219204…D695FC99` (PKID 2) |

Loading both halves (root + key) lets the verifier authenticate the stream's
DSM-PKR (via the root) *and* its DSM-KROOT (via the key) without waiting 6 h for a
DSM-PKR.

The Merkle tree was **renewed on 2024-01-15** — a new root distinct from 2023's,
which is why the GSC's *current* downloadable files don't authenticate a pre-2024
capture. The 2024 and 2025 publications share that renewed root; only the public
key rotates (PKID 1 → 2 on 2025-12-10). The 2023 PKID is **1**, not 0 — confirmed
by the signal itself: the DSM-PKR in the FGI recording carries
`new_public_key_id = 1` for `0374A925…`.

## What we verified on the FGI clean recording

Running an E1B decode over the FGI clean recording (471 s, `--sats 4,9,21,31,34,36`):

> `verified public key in DSM-PKR: DsmPkr { number_of_blocks: 13, …`
> `new_public_key_id: 1, new_public_key: Some([3, 116, 169, 37, …]) }`  ← `0374A925…`

The crate reassembled the **169-byte DSM-PKR across 13 subframes and verified its
public key against the GSC Merkle root**. An ECDSA-signed Merkle proof cannot pass
by chance, so this proves the whole real-signal path — I/NAV decode → 40-bit field
→ per-page GST → crate — is **byte-perfect**.

### The DSM-KROOT gap (no full nav-data auth from this capture)

Full nav-data authentication additionally needs a complete **DSM-KROOT** (to
derive the TESLA root key). This recording's OSNMA broadcast is **DSM-PKR-
dominated**: ~13 of its ~15 subframes carry the PKR, with only a couple of
DSM-KROOT fragments (ids 0, 2) at the start. So the KROOT never completes and the
verifier stays at `no valid TESLA key for the chain in force`. This is a property
of the captured window — a cold receiver here genuinely can't reach full auth,
whereas a warm receiver would already hold the KROOT from before the capture.

### The jammertest recording (Scenario 2) — one block short

The FGI dataset's other capture is from Jammertest 2023 (Andøya, Norway, 740 s).
Here the OSNMA broadcast *does* send DSM-KROOT (ids 4 then 5), and the fix is the
authentic Andøya location (`69.275, 15.970`) — this window is not position-spoofed.
The DSM-KROOT id 5 is **NB = 8 blocks**, and over the run the verifier received
blocks **0, 1, 2, 3, 5, 6, 7 — every block except block 4** (`missing 1 blocks`),
so the KROOT never completes and again no nav-data authentication. No
contents-differ errors (nothing forged), so all six visible SVs simply *miss the
same block*: a common-mode loss, consistent with interference in this jammertest
capture corrupting that one block's subframe (tracking holds, but a brief burst
fails the data CRC for those pages). It is not a pipeline bug — the clean
recording completed a 13-block DSM-PKR — and only those six Galileo SVs are
visible at Andøya, so there's no extra redundancy to recover block 4 from.

So both FGI captures fall one step short of full nav-data auth, for different
reasons: clean = no KROOT broadcast (PKR-dominated), jammertest = KROOT broadcast
but one block denied. To demonstrate the full TESLA/MAC step end-to-end, feed a
stream that contains a *complete* DSM-KROOT — e.g. the GSC EUSPA OSNMA test
vectors (a hex I/NAV+OSNMA stream, fed straight into `OsnmaVerifier`, no
acquire/track), or a recording from a clean KROOT-broadcasting window.

## Running it

OSNMA runs automatically on any E1B decode — there's no flag:

```sh
RUST_LOG=warn,galileo_osnma=info ./target/release/gnss-rcv \
  -f <recording> -t i8 --fs 26M --fi 6.39M --sig E1B \
  --sats 4,9,21,31,34,36
```

**Restrict `--sats` to the visible satellites.** OSNMA itself costs ~5% (measured:
60 s data, 6 sats → 22.6 s wall vs 21.5 s for the same E1B decode pre-OSNMA). The real cost
is acquisition: with all 36 PRNs, ~30 never-locking channels re-search every step
(~11× *slower* than real-time); the visible six run at **2.8× real-time**, so the
whole 471 s recording processes in ~3 min. Raise `galileo_osnma` to `debug` for
block-level DSM detail.
