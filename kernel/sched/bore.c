/*
 *  Burst-Oriented Response Enhancer (BORE) CPU Scheduler
 *  Copyright (C) 2021-2025 Masahito Suzuki <firelzrd@gmail.com>
 */

#define SCHED_BORE_AUTHOR	"Masahito Suzuki"
#define SCHED_BORE_PROGNAME	"BORE CPU Scheduler modification"
#define SCHED_BORE_VERSION	"6.1.0"

#define BORE_PENALTY_OFFSET	(25)
#define BORE_PENALTY_SCALE	(3180)
#define BORE_PENALTY_SHIFT	(12)
#define BORE_SMOOTHNESS		(40)
#define BORE_MAX_PENALTY	((40U << BORE_PENALTY_SHIFT) - 1)
#define BORE_CACHE_LIFETIME	(100000000)

#define bore_for_each_child(p, t) \
	list_for_each_entry((t), &(p)->children, sibling)

#define bore_scale_slice(delta, score) \
	mul_u64_u32_shr((delta), sched_prio_to_wmult[(score)], 22)

#define bore_unscale_slice(delta, score) \
	mul_u64_u32_shr((delta), sched_prio_to_weight[(score)], 10)

#define bore_task_is_eligible(p) \
	((p) && (p)->sched_class == &fair_sched_class && !(p)->exit_state)

#define bore_cache_expired(bc, now) \
	((s64)((bc)->timestamp + BORE_CACHE_LIFETIME - (now)) < 0)

static inline u32 log2p1_u64_u32fp(u64 v, u8 fp)
{
	if (!v)
		return 0;
	u32 exponent = fls64(v);
	u32 mantissa = (u32)(v << (64 - exponent) << 1 >> (64 - fp));
	return exponent << fp | mantissa;
}

static inline u32 calc_burst_penalty(u64 burst_time)
{
	s32 greed, tolerance, penalty, scaled_penalty;

	greed = log2p1_u64_u32fp(burst_time, BORE_PENALTY_SHIFT);
	tolerance = BORE_PENALTY_OFFSET << BORE_PENALTY_SHIFT;
	penalty = max_t(s32, 0, (greed - tolerance));
	scaled_penalty = penalty * BORE_PENALTY_SCALE >> 10;

	return min_t(u32, BORE_MAX_PENALTY, scaled_penalty);
}

static inline void reweight_task_by_prio(struct task_struct *p, int prio)
{
	struct sched_entity *se = &p->se;
	unsigned long weight = scale_load(sched_prio_to_weight[prio]);

	reweight_entity(cfs_rq_of(se), se, weight, true);
	se->load.inv_weight = sched_prio_to_wmult[prio];
}

static inline u8 effective_prio(struct task_struct *p)
{
	u8 prio = p->static_prio - MAX_RT_PRIO;
	prio += p->se.burst_score;
	return min_t(u8, 39, prio);
}

void update_burst_score(struct sched_entity *se)
{
	if (!entity_is_task(se))
		return;
	struct task_struct *p = task_of(se);
	u8 prev_prio = effective_prio(p);
	u8 burst_score = 0, new_prio;

	if (!(p->flags & PF_KTHREAD))
		burst_score = se->burst_penalty >> BORE_PENALTY_SHIFT;
	se->burst_score = burst_score;

	new_prio = effective_prio(p);
	if (new_prio != prev_prio)
		reweight_task_by_prio(p, new_prio);
}

void update_curr_bore(u64 delta_exec, struct sched_entity *se)
{
	if (!entity_is_task(se))
		return;

	se->burst_time += delta_exec;
	se->curr_burst_penalty = calc_burst_penalty(se->burst_time);
	if (se->curr_burst_penalty > se->prev_burst_penalty)
		se->burst_penalty =
			se->prev_burst_penalty +
			(se->curr_burst_penalty - se->prev_burst_penalty) /
				se->burst_count;
	update_burst_score(se);
}

static inline u32 binary_smooth(u32 new, u32 old, u8 dumper)
{
	u32 abs_diff = (new > old) ? (new - old) : (old - new);
	u32 adj_diff = (abs_diff / dumper) + ((abs_diff % dumper) != 0);
	return (new > old) ? (old + adj_diff) : (old - adj_diff);
}

static inline void __restart_burst(struct sched_entity *se)
{
	se->prev_burst_penalty = binary_smooth(se->curr_burst_penalty,
					       se->prev_burst_penalty,
					       se->burst_count);
	se->burst_time = 0;
	se->curr_burst_penalty = 0;

	if (se->burst_count < BORE_SMOOTHNESS)
		se->burst_count++;
	else
		se->burst_count = BORE_SMOOTHNESS;
}

void restart_burst(struct sched_entity *se)
{
	__restart_burst(se);
	se->burst_penalty = se->prev_burst_penalty;
	update_burst_score(se);
}

void restart_burst_rescale_deadline(struct sched_entity *se)
{
	s64 vscaled, wremain, vremain = se->deadline - se->vruntime;
	struct task_struct *p = task_of(se);
	u8 prev_prio = effective_prio(p), new_prio;

	restart_burst(se);
	new_prio = effective_prio(p);
	if (prev_prio > new_prio) {
		wremain = bore_unscale_slice(abs(vremain), prev_prio);
		vscaled = bore_scale_slice(wremain, new_prio);
		if (unlikely(vremain < 0))
			vscaled = -vscaled;
		se->deadline = se->vruntime + vscaled;
	}
}

static inline void update_burst_cache(struct sched_burst_cache *bc,
				      struct task_struct *p, u32 cnt, u32 sum,
				      u64 now)
{
	u32 avg = cnt ? sum / cnt : 0;
	bc->value = max_t(u32, avg, p->se.burst_penalty);
	bc->count = cnt;
	bc->timestamp = now;
}

static inline void update_child_burst_direct(struct task_struct *p, u64 now)
{
	u32 cnt = 0, sum = 0;
	struct task_struct *child;

	bore_for_each_child(p, child) {
		if (!bore_task_is_eligible(child))
			continue;
		cnt++;
		sum += child->se.burst_penalty;
	}

	update_burst_cache(&p->se.child_burst, p, cnt, sum, now);
}

static inline u32 inherit_burst_direct(struct task_struct *p, u64 now,
				       u64 clone_flags)
{
	struct task_struct *parent = p;
	struct sched_burst_cache *bc;

	if (clone_flags & CLONE_PARENT)
		parent = parent->real_parent;

	bc = &parent->se.child_burst;
	if (bore_cache_expired(bc, now))
		update_child_burst_direct(parent, now);

	return bc->value;
}

static inline void update_tg_burst(struct task_struct *p, u64 now)
{
	struct task_struct *task;
	u32 cnt = 0, sum = 0;

	for_each_thread(p, task) {
		if (!bore_task_is_eligible(task))
			continue;
		cnt++;
		sum += task->se.burst_penalty;
	}

	update_burst_cache(&p->se.group_burst, p, cnt, sum, now);
}

static inline u32 inherit_burst_tg(struct task_struct *p, u64 now)
{
	struct task_struct *parent = p->group_leader;
	struct sched_burst_cache *bc = &parent->se.group_burst;

	if (bore_cache_expired(bc, now))
		update_tg_burst(parent, now);

	return bc->value;
}

void sched_clone_bore(struct task_struct *p, struct task_struct *parent,
		      u64 clone_flags, u64 now)
{
	struct sched_entity *se = &p->se;
	u32 penalty;

	if (!bore_task_is_eligible(p))
		return;

	penalty = (clone_flags & CLONE_THREAD) ?
		  inherit_burst_tg(parent, now) :
		  inherit_burst_direct(parent, now, clone_flags);

	__restart_burst(se);
	se->burst_penalty = se->prev_burst_penalty =
		max_t(u32, se->prev_burst_penalty, penalty);
	se->burst_count = 1;
	se->child_burst.timestamp = 0;
	se->group_burst.timestamp = 0;
}

void reset_task_bore(struct task_struct *p)
{
	p->se.burst_time = 0;
	p->se.prev_burst_penalty = 0;
	p->se.curr_burst_penalty = 0;
	p->se.burst_penalty = 0;
	p->se.burst_score = 0;
	p->se.burst_count = 1;
	memset(&p->se.child_burst, 0, sizeof(struct sched_burst_cache));
	memset(&p->se.group_burst, 0, sizeof(struct sched_burst_cache));
}

void __init sched_bore_init(void)
{
	printk(KERN_INFO "%s %s by %s\n", SCHED_BORE_PROGNAME,
	       SCHED_BORE_VERSION, SCHED_BORE_AUTHOR);
	reset_task_bore(&init_task);
}
