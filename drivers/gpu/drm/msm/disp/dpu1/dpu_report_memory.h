/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Check-time reports a compositor's refuse-and-fall-back retry cannot repeat -- pure logic, so a host test
 * can include this header unchanged. A host build defines u32, time_before_eq() and time_after() (as
 * <linux/jiffies.h> does) and includes <stdbool.h> before including it.
 *
 * THE LOOP. A compositor proposes a client's buffer for a plane in a TEST commit every frame. When the
 * plane check refuses it, the compositor draws the frame itself and commits ITS OWN buffer to the same
 * plane -- through the same check -- and proposes the client's buffer again on the next frame. The plane's
 * checks then see two keys alternating, and a report that fires whenever its key changes fires on every one
 * of them: measured, ~89 lines a second from KWin for a 30 fps client (one retry per client frame, three
 * lines each, 2026-09-29); up to ~360 a second for a client at the full 120 Hz is an inference.
 */
#ifndef _DPU_REPORT_MEMORY_H
#define _DPU_REPORT_MEMORY_H

#ifdef __KERNEL__
#include <linux/jiffies.h>
#include <linux/types.h>
#endif

/*
 * How long a key the plane has left is remembered. KWin retried a refused plane 119.7 times a second at
 * 120 Hz (measured 2026-09-29), one retry every ~8 ms; the fastest reader of these lines returns a plane to
 * a format every 8 s (a fullscreen-toggle probe). One second is two orders of magnitude from both.
 */
#define DPU_REPORT_WINDOW_MS	1000

/**
 * struct dpu_report_change - memory for a report that says when a plane's key CHANGES
 * @cur:       the key at the plane's last check -- the single entry these reports always kept.
 *             dpu_report_change_note() never expires it, so a key held across an idle gap is never
 *             re-reported; dpu_report_state_note() expires it after the window
 * @prev:      the key before that
 * @cur_seen:  when @cur was last seen, in jiffies
 * @prev_seen: when @prev was last seen, in jiffies
 * @returns:   returns to @prev within the window since the last new key, saturating at 2
 * @has_cur:   @cur holds a key
 * @has_prev:  @prev holds a key
 */
struct dpu_report_change {
	u32 cur, prev;
	unsigned long cur_seen, prev_seen;
	unsigned int returns;
	bool has_cur, has_prev;
};

/**
 * dpu_report_change_note - note one check's key and say whether to report it
 * @m:      the plane's memory for this report
 * @key:    the key the check saw
 * @now:    when, in jiffies
 * @window: how long a key the plane has left is remembered, in jiffies
 *
 * @key is reported when it differs from the key at the plane's last check -- the single-entry memory's
 * rule -- UNLESS it is the plane's previous distinct key, last seen at a check on this plane no more than
 * @window before, and the plane has already come back to that key once since its last new key. So one
 * refused offer still reports the offer and the fall-back; a two-key refuse-and-fall-back loop reports
 * those two lines and then nothing while its retries come no more than @window apart; a key that comes back
 * after @window is reported as new. It never reports a key the single-entry memory would not have.
 *
 * Its limits, exactly: the key at the plane's last check is silent at ANY age, as it always was; beyond
 * that, nothing is silenced on a plane checked less often than once per @window (a refused client slower
 * than that is reported every frame, as before); an episode that begins more than @window after the
 * plane's last check reports its first return as a new key, one line more; and only TWO keys are held -- a
 * cycle through three reports every change.
 *
 * Return: true if the caller should report @key.
 */
static inline bool dpu_report_change_note(struct dpu_report_change *m, u32 key,
					  unsigned long now, unsigned long window)
{
	unsigned long t;
	u32 k;

	if (m->has_cur && key == m->cur) {
		m->cur_seen = now;
		return false;
	}

	if (m->has_prev && key == m->prev && time_before_eq(now, m->prev_seen + window)) {
		k = m->cur;
		t = m->cur_seen;
		m->cur = m->prev;
		m->cur_seen = now;
		m->prev = k;
		m->prev_seen = t;
		if (m->returns < 2)
			m->returns++;
		return m->returns == 1;
	}

	m->prev = m->cur;
	m->prev_seen = m->cur_seen;
	m->has_prev = m->has_cur;
	m->cur = key;
	m->cur_seen = now;
	m->has_cur = true;
	m->returns = 0;
	return true;
}

/**
 * dpu_report_state_note - dpu_report_change_note() for a report that ALSO needs its key to differ from the
 * plane's committed state before it prints
 * @m:      the plane's memory for this report
 * @key:    the key the check saw
 * @now:    when, in jiffies
 * @window: how long a key is remembered, in jiffies
 *
 * The change memory never expires its current key: that is what keeps an idle plane quiet when nothing else
 * stands between the report and the log. A report that is also held back by the committed state has
 * something else, so here the current key expires too, after @window. It matters on a plane whose fall-back
 * never passes this check -- one the compositor switches off, or leaves out of the commit, when the kernel
 * refuses it: the memory then keeps the refused offer's key, and with no expiry an offer ACCEPTED later with
 * the same key would never be reported.
 *
 * So a silence here always means this plane checked @key no more than @window before, and with the
 * caller's committed-state test "no line" means @key matched the committed state or was checked on this
 * plane in the preceding @window. The converse does not hold: a first return within @window is reported.
 *
 * Return: true if the caller may report @key; the caller still requires it to differ from the committed
 * state.
 */
static inline bool dpu_report_state_note(struct dpu_report_change *m, u32 key,
					 unsigned long now, unsigned long window)
{
	bool stale = m->has_cur && key == m->cur && time_after(now, m->cur_seen + window);

	return dpu_report_change_note(m, key, now, window) || stale;
}

/**
 * struct dpu_report_event - memory for a report of a refusal, which happens rather than holds
 * @key:  the refusal last noted
 * @seen: when, in jiffies
 * @has:  @key holds one
 */
struct dpu_report_event {
	u32 key;
	unsigned long seen;
	bool has;
};

/**
 * dpu_report_event_note - note one refusal and say whether to report it
 * @m:      the plane's refusal memory
 * @key:    which refusal
 * @now:    when, in jiffies
 * @window: how long a run of the same refusal lasts without it recurring, in jiffies
 *
 * Once per RUN: a refusal is reported unless the same one was noted no more than @window earlier, so a
 * compositor retrying a refused plane every frame gets one line for the whole run and a later run gets its
 * own. One entry: two different refusals alternating are each reported every time.
 *
 * Return: true if the caller should report @key.
 */
static inline bool dpu_report_event_note(struct dpu_report_event *m, u32 key,
					 unsigned long now, unsigned long window)
{
	bool repeat = m->has && key == m->key && time_before_eq(now, m->seen + window);

	m->key = key;
	m->seen = now;
	m->has = true;
	return !repeat;
}

#endif /* _DPU_REPORT_MEMORY_H */
