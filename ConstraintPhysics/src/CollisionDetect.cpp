#include "CollisionDetect.h"

namespace phyz {
	struct SATNormcheckThread;

	static struct CheckNormResults {
		mthz::Vec3 a_maxP;
		mthz::Vec3 b_maxP;
		double pen_depth;

		inline bool seprAxisExists() { return pen_depth < 0; }
	};

	static struct NormCheckJob {
		const ConvexPoly* a;
		const ConvexPoly* b;
		const std::vector<mthz::Vec3>* norms;
		std::vector<CheckNormResults>* outs;
		int start_indx;
		int job_size;
		bool* job_finished_flag;
		bool* early_termination_flag;
	};

	static void threadFunc();
	static CheckNormResults sat_checknorm(const ConvexPoly& a, const ConvexPoly& b, mthz::Vec3 n);

	static int n_threads = 0;
	static std::vector<std::thread> thread_pool;
	static const int QUEUE_SIZE = 64;
	//static const int MAX_JOB_SIZE = 32;
	static NormCheckJob job_queue[QUEUE_SIZE];
	static std::atomic_int num_queued;
	static std::atomic_int job_submit_indx;
	static std::atomic_int job_consume_indx;
	static std::mutex job_queue_lock;
	static bool inited = false;
	static bool kill_all_threads = false;

	void init_multithreaded_sat(int n_helper_threads) {
		if (!inited) {
			inited = true;
			kill_all_threads = false;
			n_threads = n_helper_threads;
			num_queued = 0;
			job_submit_indx = 0;
			job_consume_indx = 0;
			for (int i = 0; i < n_helper_threads; i++) {
				thread_pool.push_back(std::thread(threadFunc));
			}
		}
	}

	static void threadFunc() {
		while (!kill_all_threads) {
			job_queue_lock.lock();

			if (num_queued.load() == 0) {
				job_queue_lock.unlock();
				std::this_thread::yield();
				continue;
			}

			NormCheckJob job = job_queue[job_consume_indx];
			job_consume_indx = (job_consume_indx + 1) % QUEUE_SIZE;
			num_queued--;

			job_queue_lock.unlock();

			for (int i = job.start_indx; i < job.start_indx + job.job_size && (job.early_termination_flag == nullptr || !(*job.early_termination_flag)); i++) {
				job.outs->at(i) = sat_checknorm(*job.a, *job.b, job.norms->at(i));
				if (job.outs->at(i).seprAxisExists() && job.early_termination_flag != nullptr) { 
					*job.early_termination_flag = true; 
				}
			}

			if (job.job_finished_flag != nullptr) { 
				*job.job_finished_flag = true; 
			}

			
		}
	}

	static void submitJob(const ConvexPoly& a, const ConvexPoly& b, const std::vector<mthz::Vec3>& norms, std::vector<CheckNormResults>* outs, int start_indx, int job_size, bool* job_completion_flag=nullptr, bool* early_termination_flag=nullptr) {
		do {
			job_queue_lock.lock();

			if (num_queued.load() == QUEUE_SIZE) {
				job_queue_lock.unlock();
				std::this_thread::yield();
				continue;
			}
		} while (false);

		int job_indx = job_submit_indx;
		job_submit_indx = (job_submit_indx + 1) % QUEUE_SIZE;

		job_queue[job_indx] = { &a, &b, &norms, outs, start_indx, job_size, job_completion_flag, early_termination_flag};
		num_queued++;

		job_queue_lock.unlock();
		
	}

	static struct ProjP {
		double u;
		double w;
	};

	//kinda brute forcey might redo later
	static std::vector<ProjP> findContactArea(const ConvexPoly& c, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w) {

		std::vector<ProjP> out;

		for (const Surface& s : c.surfaces) {
			if (1 - s.normal().dot(n) <= COS_TOL) {
				int n_points = s.n_points();
				out = std::vector<ProjP>(n_points);
				for (int i = 0; i < n_points; i++) {
					mthz::Vec3 v = s.getPointI(i);
					out[i] = ProjP{ v.dot(u), v.dot(w) };
				}
				return out;
			}
		}

		for (Edge e : c.edges) {
			mthz::Vec3 p1 = e.p1();
			mthz::Vec3 p2 = e.p2();
			double d = abs((e.p2() - e.p1()).normalize().dot(n));
			if ((e.p1() == p || e.p2() == p) && abs((e.p2() - e.p1()).normalize().dot(n)) <= SIN_TOL) {
				out = { ProjP{ e.p1().dot(u), e.p1().dot(w) }, ProjP{ e.p2().dot(u), e.p2().dot(w) } };
				return out;
			}
		}

		return out = { ProjP{ p.dot(u), p.dot(w) } };
	}

	static bool pInside(const std::vector<ProjP> poly, ProjP p) {
		int left = 0;
		int right = 0;

		for (int i = 0; i < poly.size(); i++) {
			ProjP v1 = poly[i];
			ProjP v2 = poly[(i + 1) % poly.size()];


			if (v1.w == p.w) {
				//c2 will be checked for this check next iteration
				if (v1.u > p.u) {
					right++;
				}
				else {
					left++;
				}
			}
			else if ((v1.w - p.w) * (v2.w - p.w) < 0) {
				double intr_u = v1.u + (p.w - v1.w) * (v2.u - v1.u) / (v2.w - v1.w);
				if (intr_u > p.u) {
					right++;
				}
				else {
					left++;
				}
			}
		}

		return (left % 2 != 0) && (right % 2 != 0);
	}

	static bool findIntersection(ProjP a1, ProjP a2, ProjP b1, ProjP b2, ProjP* out) {
		double ma = (a2.w - a1.w) / (a2.u - a1.u);
		double mb = (b2.w - b1.w) / (b2.u - b1.u);

		//edge cases
		if (a1.u == a2.u) {
			double du = a1.u - b1.u;
			double wIntr = b1.w + du * mb;
			if ((b1.u - a1.u) * (b2.u - a1.u) <= 0 && (a1.w - wIntr) * (a2.w - wIntr) <= 0) {
				*out = ProjP{ a1.u, b1.w + (a1.u - b1.u) * (b2.w - b1.w) / (b2.u - b1.u) };
				return true;
			}
			return false;
		}
		if (b1.u == b2.u) {
			double du = b1.u - a1.u;
			double wIntr = a1.w + du * ma;
			if ((a1.u - b1.u) * (a2.u - b1.u) <= 0 && (b1.w - wIntr) * (b2.w - wIntr) <= 0) {
				*out = ProjP{ b1.u, a1.w + (b1.u - a1.u) * (a2.w - a1.w) / (a2.u - a1.u) };
				return true;
			}
			return false;
		}

		double dw0 = b1.w + mb * (a1.u - b1.u) - a1.w;
		double du = dw0 / (ma - mb);
		*out = ProjP{ a1.u + du, a1.w + ma * du };

		//check intersection is in bounds
		if ((out->u - a1.u) * (out->u - a2.u) >= 0 || (out->u - b1.u) * (out->u - b2.u) >= 0) {
			return false;
		}
		return true;
	}

	static struct ExtremaInfo {
		ExtremaInfo() {
			min_val = std::numeric_limits<double>::infinity();
			max_val = -std::numeric_limits<double>::infinity();
		}

		mthz::Vec3 min_p;
		mthz::Vec3 max_p;
		double min_val;
		double max_val;
	};

	static ExtremaInfo findExtrema(const ConvexPoly& c, mthz::Vec3 axis) {
		ExtremaInfo extrema;

		for (mthz::Vec3 p : c.points) {
			double val = p.dot(axis);
			if (val < extrema.min_val) {
				extrema.min_p = p;
				extrema.min_val = val;
			}
			if (val > extrema.max_val) {
				extrema.max_p = p;
				extrema.max_val = val;
			}
		}

		return extrema;
	}

	static CheckNormResults sat_checknorm(const ConvexPoly& a, const ConvexPoly& b, mthz::Vec3 n) {
		ExtremaInfo e1 = findExtrema(a, n);
		ExtremaInfo e2 = findExtrema(b, n);

		return { e1.max_p, e2.min_p, e1.max_val - e2.min_val };
	}

	Manifold SAT(const ConvexPoly& a, const RigidBody::GaussMap& ag, const ConvexPoly& b, const RigidBody::GaussMap& bg) {
		Manifold out;
		bool sepr_axis = false; //assumed false at first
		bool first_check = true;
		mthz::Vec3 norm;
		mthz::Vec3 a_maxP, b_maxP;
		double pen_depth = std::numeric_limits<double>::infinity();

		std::vector<mthz::Vec3> norms_to_check;
		norms_to_check.reserve(ag.face_verts.size() + bg.face_verts.size() + ag.arcs.size() * bg.arcs.size() / 4);
		for (mthz::Vec3 n : ag.face_verts) {
			norms_to_check.push_back(n);
		}
		for (mthz::Vec3 n : bg.face_verts) {
			norms_to_check.push_back(n);
		}
		for (RigidBody::GaussArc arc1 : ag.arcs) {
			for (RigidBody::GaussArc arc2 : bg.arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx];
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx];
				mthz::Vec3 b1 = -bg.face_verts[arc2.v1_indx];
				mthz::Vec3 b2 = -bg.face_verts[arc2.v2_indx];

				//check arcs arent on opposite hemispheres
				if ((a1 + a2).dot(b1 + b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if ((a1 + a2).dot(n) < 0) {
					n *= -1;
				}
				norms_to_check.push_back(n);
			}
		}

		//printf("%d\n", norms_to_check.size());

		if (n_threads > 0) {
			int n_jobs = n_threads;
			int job_size = norms_to_check.size() / n_threads;
			std::vector<CheckNormResults> normcheck_outputs(norms_to_check.size());
			bool sepr_axis_exists = false;
			std::vector<char> job_finished(n_jobs, false); //need to to shenanigans as vector of bool is a bit vector, so cant take address of

			for (int i = 0; i < n_jobs; i++) {
				int start_indx = i * job_size;
				int this_job_size = (i + 1 == n_jobs) ? job_size + (norms_to_check.size() % job_size) : job_size;
				submitJob(a, b, norms_to_check, &normcheck_outputs, start_indx, this_job_size, (bool*)&job_finished[i], &sepr_axis_exists);
			}
			for (int i = 0; i < job_finished.size(); i++) {
				while (!job_finished[i]);
			}

			if (sepr_axis_exists) {
				out.pen_depth = -1;
				return out;
			}

			for (int i = 0; i < norms_to_check.size(); i++) {
				const CheckNormResults& x = normcheck_outputs[i];
				if (x.pen_depth < pen_depth) {
					pen_depth = x.pen_depth;
					a_maxP = x.a_maxP;
					b_maxP = x.b_maxP;
					norm = norms_to_check[i];
				}
			}
		}
		else {
			for (mthz::Vec3 n : norms_to_check) {
				CheckNormResults x = sat_checknorm(a, b, n);
				if (x.seprAxisExists()) {
					out.pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < pen_depth) {
					pen_depth = x.pen_depth;
					a_maxP = x.a_maxP;
					b_maxP = x.b_maxP;
					norm = n;
				}
			}
		}


		out.pen_depth = pen_depth;
		out.normal = norm;

		//generate manifold
		//make arbitrary perp vector
		const mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(norm.dot(axis1)) < abs(norm.dot(axis2))) ? norm.cross(axis1).normalize() : norm.cross(axis2).normalize();
		mthz::Vec3 w = norm.cross(u);
		std::vector<ProjP> a_contact = findContactArea(a, norm, a_maxP, u, w);
		std::vector<ProjP> b_contact = findContactArea(b, norm * (-1), b_maxP, u, w);

		std::vector<ProjP> man_pool;
		for (ProjP p : a_contact) {
			if (pInside(b_contact, p)) {
				man_pool.push_back(p);
			}
		}
		for (ProjP p : b_contact) {
			if (pInside(a_contact, p)) {
				man_pool.push_back(p);
			}
		}
		if (a_contact.size() >= 2 && b_contact.size() >= 2) {
			int itr_max_a = (a_contact.size() == 2) ? 1 : a_contact.size();
			int itr_max_b = (b_contact.size() == 2) ? 1 : b_contact.size();
			for (int i = 0; i < itr_max_a; i++) {
				ProjP a1 = a_contact[i];
				ProjP a2 = a_contact[(i + 1) % a_contact.size()];

				for (int j = 0; j < itr_max_b; j++) {
					ProjP b1 = b_contact[j];
					ProjP b2 = b_contact[(j + 1) % b_contact.size()];

					ProjP out;
					if (findIntersection(a1, a2, b1, b2, &out)) {
						man_pool.push_back(out);
					}
				}
			}
		}

		mthz::Vec3 n_offset = norm * a_maxP.dot(norm);

		for (ProjP p : man_pool) {
			out.points.push_back(u * p.u + w * p.w + n_offset);
		}

		return out;
	}

	/*Manifold SAT(const ConvexPoly& a, const RigidBody::GaussMap& ag, const ConvexPoly& b, const RigidBody::GaussMap& bg) {
		Manifold out;
		bool sepr_axis = false; //assumed false at first
		bool first_check = true;
		mthz::Vec3 norm;
		mthz::Vec3 a_maxP, b_maxP;
		double pen_depth = std::numeric_limits<double>::infinity();

		for (mthz::Vec3 n : ag.face_verts) {
			CheckNormResults x = sat_checknorm(a, b, n);
			if (x.seprAxisExists()) {
				out.pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < pen_depth) {
				pen_depth = x.pen_depth;
				a_maxP = x.a_maxP;
				b_maxP = x.b_maxP;
				norm = n;
			}
		}

		for (mthz::Vec3 n : bg.face_verts) {
			CheckNormResults x = sat_checknorm(a, b, n);
			if (x.seprAxisExists()) {
				out.pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < pen_depth) {
				pen_depth = x.pen_depth;
				a_maxP = x.a_maxP;
				b_maxP = x.b_maxP;
				norm = n;
			}
		}

		for (RigidBody::GaussArc arc1 : ag.arcs) {
			for (RigidBody::GaussArc arc2 : bg.arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx];
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx];
				mthz::Vec3 b1 = -bg.face_verts[arc2.v1_indx];
				mthz::Vec3 b2 = -bg.face_verts[arc2.v2_indx];

				//check arcs arent on opposite hemispheres
				if (((a1 + a2) / 2).dot((b1 + b2) / 2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if ((a1 + a2).dot(n) < 0) {
					n *= -1;
				}

				CheckNormResults x = sat_checknorm(a, b, n);
				if (x.seprAxisExists()) {
					out.pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < pen_depth) {
					pen_depth = x.pen_depth;
					a_maxP = x.a_maxP;
					b_maxP = x.b_maxP;
					norm = n;
				}
			}
		}

		out.pen_depth = pen_depth;
		out.normal = norm;

		//generate manifold
		//make arbitrary perp vector
		const mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(norm.dot(axis1)) < abs(norm.dot(axis2))) ? norm.cross(axis1).normalize() : norm.cross(axis2).normalize();
		mthz::Vec3 w = norm.cross(u);
		std::vector<ProjP> a_contact = findContactArea(a, norm, a_maxP, u, w);
		std::vector<ProjP> b_contact = findContactArea(b, norm * (-1), b_maxP, u, w);

		std::vector<ProjP> man_pool;
		for (ProjP p : a_contact) {
			if (pInside(b_contact, p)) {
				man_pool.push_back(p);
			}
		}
		for (ProjP p : b_contact) {
			if (pInside(a_contact, p)) {
				man_pool.push_back(p);
			}
		}
		if (a_contact.size() >= 2 && b_contact.size() >= 2) {
			int itr_max_a = (a_contact.size() == 2) ? 1 : a_contact.size();
			int itr_max_b = (b_contact.size() == 2) ? 1 : b_contact.size();
			for (int i = 0; i < itr_max_a; i++) {
				ProjP a1 = a_contact[i];
				ProjP a2 = a_contact[(i + 1) % a_contact.size()];

				for (int j = 0; j < itr_max_b; j++) {
					ProjP b1 = b_contact[j];
					ProjP b2 = b_contact[(j + 1) % b_contact.size()];

					ProjP out;
					if (findIntersection(a1, a2, b1, b2, &out)) {
						man_pool.push_back(out);
					}
				}
			}
		}

		mthz::Vec3 n_offset = norm * a_maxP.dot(norm);

		for (ProjP p : man_pool) {
			out.points.push_back(u * p.u + w * p.w + n_offset);
		}

		return out;
	}*/

	Manifold merge_manifold(const Manifold& m1, const Manifold& m2) {
		Manifold out = { std::vector<mthz::Vec3>(m1.points.size() + m2.points.size()), mthz::Vec3(), -1 };
		if (m1.pen_depth > m2.pen_depth) {
			out.normal = m1.normal;
			out.pen_depth = m1.pen_depth;
		}
		else {
			out.normal = m2.normal;
			out.pen_depth = m2.pen_depth;
		}

		for (int i = 0; i < m1.points.size(); i++) {
			out.points[i] = m1.points[i];
		}
		int off = m1.points.size();
		for (int i = 0; i < m2.points.size(); i++) {
			out.points[i + off] = m2.points[i];
		}

		return out;
	}

	Manifold cull_manifold(const Manifold& m, int new_size) {
		if (new_size >= m.points.size()) {
			return m;
		}
		Manifold out = { std::vector<mthz::Vec3>(new_size), m.normal, m.pen_depth };
		std::vector<bool> p_available(m.points.size(), true);

		const mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(m.normal.dot(axis1)) < abs(m.normal.dot(axis2))) ? m.normal.cross(axis1).normalize() : m.normal.cross(axis2).normalize();
		mthz::Vec3 w = m.normal.cross(u);

		for (int i = 0; i < new_size; i++) {
			double vu = cos(2 * M_PI * i / new_size);
			double vw = sin(2 * M_PI * i / new_size);
			mthz::Vec3 target_dir = u * vu + w * vw;

			mthz::Vec3 max_p;
			int max_indx;
			double max_v = -std::numeric_limits<double>::infinity();
			for (int j = 0; j < m.points.size(); j++) {
				if (p_available[j]) {
					mthz::Vec3 p = m.points[j];
					double val = p.dot(target_dir);
					if (val > max_v) {
						max_v = val;
						max_p = p;
						max_indx = j;
					}
				}
			}
			out.points[i] = max_p;
			p_available[max_indx] = false;
		}

		return out;
	}
}