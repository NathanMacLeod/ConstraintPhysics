#include "HolonomicBlockSolver.h"
#include "ConstraintSolver.h"
#include <map>

namespace phyz {

	static int get_edge_map_index(int num_constraints, int c1_index, int c2_index) {
		return c1_index + c2_index * num_constraints;
	}

	HolonomicSystem::HolonomicSystem(std::vector<Constraint*> constraints_og_order)
		: system_degree(0), buffer_capacity(0)
	{
		int n = constraints_og_order.size();
		if (constraints_og_order.size() == 0) return;
		//compute the proper ordering to have the most sparse system possible. Track which blocks we can ignore due to sparsity.
		std::vector<bool> edge_map(n * n, false); //false means no edge
		std::map<Constraint*, std::map<Constraint*, bool>> block_stays_empty;
		std::vector<int> remaining_elimination_candidates(n);
		//initialize edge_map which we use for finding the correct elimination order, and 
		//block_stays_empty, which tracks which blocks we wil be able to ignore given our elimination order
		for (int i = 0; i < n; i++) {
			remaining_elimination_candidates[i] = i;
			for (int j = 0; j < n; j++) {
				Constraint* c1 = constraints_og_order[i];
				Constraint* c2 = constraints_og_order[j];

				assert(c1->a != nullptr && c1->b != nullptr);
				if (c1->a == c2->a || c1->a == c2->b || c1->b == c2->a || c1->b == c2->b) {
					block_stays_empty[c1][c2] = false;
					block_stays_empty[c2][c1] = false;
					edge_map[get_edge_map_index(n, i, j)] = true;
					edge_map[get_edge_map_index(n, j, i)] = true;
				}
				else {
					//assume true, might be overwritten to false later.
					block_stays_empty[c1][c2] = true;
					block_stays_empty[c2][c1] = true;
				}
			}
		}

		while (!remaining_elimination_candidates.empty()) {
			//find the next node to remove that creates the least new edges
			int least_unconnected_neighbors_index = -1;
			int least_unconnected_neighbors_count = INT_MAX;
			std::vector<int> least_unconnected_neighbors;
			for (int i = 0; i < remaining_elimination_candidates.size(); i++) {
				//get all neighbors to constraint_i
				std::vector<int> neighbors;
				for (int j = 0; j < n; j++) {
					if (j != remaining_elimination_candidates[i] && edge_map[get_edge_map_index(n, remaining_elimination_candidates[i], j)]) {
						neighbors.push_back(j);
					}
				}

				//count number of neighbors not connected to each other
				int unconnected_neighbor_count = 0;
				for (int ni = 0; ni < neighbors.size(); ni++) {
					for (int nj = ni + 1; nj < neighbors.size(); nj++) {
						if (!edge_map[get_edge_map_index(n, neighbors[ni], neighbors[nj])]) unconnected_neighbor_count++;
					}
				}

				if (unconnected_neighbor_count < least_unconnected_neighbors_count) {
					least_unconnected_neighbors_count = unconnected_neighbor_count;
					least_unconnected_neighbors_index = i;
					least_unconnected_neighbors = neighbors;
					if (unconnected_neighbor_count == 0) break; //not going to get better than 0
				}
			}

			assert(least_unconnected_neighbors_index != -1);
			//save ordering by pushing to constaints in the order of elimination
			int least_unconnected = remaining_elimination_candidates[least_unconnected_neighbors_index];
			constraints.push_back(constraints_og_order[least_unconnected]);
			remaining_elimination_candidates.erase(remaining_elimination_candidates.begin() + least_unconnected_neighbors_index);
			//remove node from graph
			for (int j = 0; j < n; j++) {
				edge_map[get_edge_map_index(n, least_unconnected, j)] = false;
				edge_map[get_edge_map_index(n, j, least_unconnected)] = false;
			}
			//create edges among neighbors
			for (int ni = 0; ni < least_unconnected_neighbors.size(); ni++) {
				for (int nj = ni + 1; nj < least_unconnected_neighbors.size(); nj++) {
					int ni_indx = least_unconnected_neighbors[ni];
					int nj_indx = least_unconnected_neighbors[nj];

					block_stays_empty[constraints_og_order[ni_indx]][constraints_og_order[nj_indx]] = false;
					block_stays_empty[constraints_og_order[nj_indx]][constraints_og_order[ni_indx]] = false;
					edge_map[get_edge_map_index(n, ni_indx, nj_indx)] = true;
					edge_map[get_edge_map_index(n, nj_indx, ni_indx)] = true;
				}
			}
		}

		//compute the required size of the buffer and fill in the block location table
		for (int col = 0; col < constraints.size(); col++) {

			vector_location_lookup.push_back(system_degree);
			system_degree += constraints[col]->getDegree();

			for (int row = col; row < constraints.size(); row++) {
				if (block_stays_empty[constraints[row]][constraints[col]]) {
					block_location_table.push_back(BLOCK_EMPTY);
				}
				else {
					block_location_table.push_back(buffer_capacity);
					buffer_capacity += constraints[row]->getDegree() * constraints[col]->getDegree();
				}
			}
		}

		int MAX_CONSTRAINT_DEGREE = 6;
		diagonal_elem_buffer_capacity = MAX_CONSTRAINT_DEGREE * system_degree;

#ifdef NDEBUG
		buffer = new double[buffer_capacity + diagonal_elem_buffer_capacity];
		diagonal_elem_buffer = buffer + buffer_capacity;
#else
		//so that contents of buffer can be viewed in the debugger
		if (USE_GAUSS_ELIM_FOR_INVERSE) {
			debug_inverse = std::vector<double>(system_degree * system_degree);
		}
		else {
			debug_buffer = std::vector<double>(buffer_capacity, 0);
			debug_diagonal_elem_buffer = std::vector<double>(diagonal_elem_buffer_capacity, 0);
		}
		buffer = debug_buffer.data();
		diagonal_elem_buffer = debug_diagonal_elem_buffer.data();
#endif
	}

	HolonomicSystem::~HolonomicSystem() {
		if (constraints.size() > 0) {
#ifdef NDEBUG
			delete buffer;
#endif
		}
	}

	HolonomicSystem::HolonomicSystem(HolonomicSystem&& h)
		: constraints(h.constraints), system_degree(h.system_degree), buffer(h.buffer), buffer_capacity(h.buffer_capacity), 
		diagonal_elem_buffer(h.diagonal_elem_buffer), diagonal_elem_buffer_capacity(h.diagonal_elem_buffer_capacity), 

		block_location_table(std::move(h.block_location_table)), vector_location_lookup(std::move(h.vector_location_lookup))
#ifndef NDEBUG
		,debug_buffer(std::move(h.debug_buffer)), debug_diagonal_elem_buffer(std::move(h.debug_diagonal_elem_buffer)), debug_inverse(std::move(h.debug_inverse))
#endif
	{
		h.buffer = nullptr;
		h.diagonal_elem_buffer = nullptr;
	}

	void HolonomicSystem::operator=(HolonomicSystem&& h) {
		this->~HolonomicSystem();
		
		constraints = std::move(h.constraints);
		system_degree = h.system_degree;
		buffer = h.buffer;
		buffer_capacity = h.buffer_capacity;
		diagonal_elem_buffer = h.diagonal_elem_buffer;
		diagonal_elem_buffer_capacity = h.diagonal_elem_buffer_capacity;
		block_location_table = std::move(h.block_location_table);
		vector_location_lookup = std::move(h.vector_location_lookup);
#ifndef NDEBUG
		debug_buffer = std::move(h.debug_buffer);
		debug_diagonal_elem_buffer = std::move(h.debug_diagonal_elem_buffer);
		debug_inverse = std::move(h.debug_inverse);
#endif
		h.buffer = nullptr;
		h.diagonal_elem_buffer = nullptr;
	}

	template<int n>
	static void writeTargetDelta(DegreedConstraint<n>* constraint, std::vector<double>* target, int write_index, bool use_psuedo_values);

	static void multMatWithVec(int block_width, int block_height, std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source);
	static void multMatTransposedWithVec(int block_width, int block_height, std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source);

	template<int n>
	static void applyImpulseChange(DegreedConstraint<n>* constraint, const std::vector<double>& impulse_source, int source_index, bool use_psuedo_values) {
		mthz::NVec<n> impulse_change;
		for (int i = 0; i < n; i++) impulse_change.v[i] = impulse_source[source_index + i];

		if (!impulse_change.isZero()) {
			mthz::NVec<6>* vel_a_change = use_psuedo_values? constraint->a_psuedo_velocity_change : constraint->a_velocity_change;
			mthz::NVec<6>* vel_b_change = use_psuedo_values? constraint->b_psuedo_velocity_change : constraint->b_velocity_change;
			mthz::NVec<n>* accumulated_impulse = use_psuedo_values? &constraint->psuedo_impulse : &constraint->impulse;

			constraint->computeAndApplyVelocityChange(impulse_change, vel_a_change, vel_b_change);
			*accumulated_impulse += impulse_change;
		}
	}

	void HolonomicSystem::computeAndApplyImpulses(bool use_psuedo_values) {
		//solving system d = A^-1(t - c)
		//d: delta in impulses needed to satisfy all constraints
		//A^1: inverse of the impulse to value matrix of this holonomic system
		//t: target value of all constraints
		//c: current value of all constraints

		std::vector<double> delta(system_degree); //initialize as t - c, then transform into d via multiplication by A^-1

		for (int i = 0; i < constraints.size(); i++) {
			int pos = getVectorPos(i);
			switch (constraints[i]->getDegree()) {
			case 1: writeTargetDelta<1>((DegreedConstraint<1>*)constraints[i], &delta, pos, use_psuedo_values); break;
			case 2: writeTargetDelta<2>((DegreedConstraint<2>*)constraints[i], &delta, pos, use_psuedo_values); break;
			case 3: writeTargetDelta<3>((DegreedConstraint<3>*)constraints[i], &delta, pos, use_psuedo_values); break;
			case 4: writeTargetDelta<4>((DegreedConstraint<4>*)constraints[i], &delta, pos, use_psuedo_values); break;
			case 5: writeTargetDelta<5>((DegreedConstraint<5>*)constraints[i], &delta, pos, use_psuedo_values); break;
			case 6: writeTargetDelta<6>((DegreedConstraint<6>*)constraints[i], &delta, pos, use_psuedo_values); break;
			}
		}
		
#ifndef NDEBUG
		if (USE_GAUSS_ELIM_FOR_INVERSE) {
			std::vector<double> correct_impulse(system_degree, 0);

			for (int i = 0; i < system_degree; i++) {
				for (int j = 0; j < system_degree; j++) {
					correct_impulse[i] += delta[j] * debug_inverse[j + system_degree * i];
				}
			}

			delta = correct_impulse;
			goto ApplyImpulse;
		}
		{
#endif

		//A^-1 = (L^-t)(D^-1)(L^-1) 
		//Multiply by L inverse (inversion is as simple as making non diagonal blocks negative)
		std::vector<double> multByLEffect(system_degree);
		for (int col = 0; col + 1 < constraints.size(); col++) {
			multByLEffect = std::vector<double>(system_degree, 0); //set to zero

			for (int row = col + 1; row < constraints.size(); row++) {
				int loc = getBlockBufferLocation(row, col);
				if (loc == BLOCK_EMPTY) continue;

				int block_width = constraints[col]->getDegree();
				int block_height = constraints[row]->getDegree();
				double* block_pos = buffer + loc;
				int source_index = getVectorPos(col);
				int write_index = getVectorPos(row);

				multMatWithVec(block_width, block_height, &multByLEffect, write_index, delta, source_index, block_pos);
			}

			for (int i = 0; i < system_degree; i++) {
				delta[i] -= multByLEffect[i];
			}
		}

		//Multiply by D inverse
		std::vector<double> multByDOut(system_degree);
		for (int col = 0; col < constraints.size(); col++) {
			int block_degree = constraints[col]->getDegree();
			double* block_pos = buffer + getBlockBufferLocation(col, col);
			int vec_index = getVectorPos(col);
			multMatWithVec(block_degree, block_degree, &multByDOut, vec_index, delta, vec_index, block_pos);
		}
		delta = multByDOut;

		//Multiply by L^-t

		std::vector<double> multByLTEffect(6);
		for (int col = constraints.size() - 2; col >= 0; col--) {
			multByLTEffect = std::vector<double>(6, 0);

			for (int row = col + 1; row < constraints.size(); row++) {
				int loc = getBlockBufferLocation(row, col);
				if (loc == BLOCK_EMPTY) continue;

				int block_width = constraints[col]->getDegree();
				int block_height = constraints[row]->getDegree();
				double* block_pos = buffer + loc;
				int source_index = getVectorPos(row);

				multMatTransposedWithVec(block_width, block_height, &multByLTEffect, 0, delta, source_index, block_pos);
			}

			int write_pos = getVectorPos(col);
			for (int i = 0; i < constraints[col]->getDegree(); i++) {
				delta[write_pos + i] -= multByLTEffect[i];
			}
		}

#ifndef NDEBUG
		}
		ApplyImpulse:
#endif

		//delta now equal to A^-1(t - c) = d, just need to store impulse and velocity changes now
		for (int i = 0; i < constraints.size(); i++) {
			int pos = getVectorPos(i);
			switch (constraints[i]->getDegree()) {
			case 1: applyImpulseChange<1>((DegreedConstraint<1>*)constraints[i], delta, pos, use_psuedo_values); break;
			case 2: applyImpulseChange<2>((DegreedConstraint<2>*)constraints[i], delta, pos, use_psuedo_values); break;
			case 3: applyImpulseChange<3>((DegreedConstraint<3>*)constraints[i], delta, pos, use_psuedo_values); break;
			case 4: applyImpulseChange<4>((DegreedConstraint<4>*)constraints[i], delta, pos, use_psuedo_values); break;
			case 5: applyImpulseChange<5>((DegreedConstraint<5>*)constraints[i], delta, pos, use_psuedo_values); break;
			case 6: applyImpulseChange<6>((DegreedConstraint<6>*)constraints[i], delta, pos, use_psuedo_values); break;
			}
		}

	}


	int HolonomicSystem::getVectorPos(int constraint_row) {
		assert(constraint_row >= 0 && constraint_row < constraints.size());
		return vector_location_lookup[constraint_row];
	}

	template<int c1_degree, int c2_degree>
	static void matMult(double* target, const mthz::NMat<c1_degree, 6>& c1_mat, const mthz::NMat<6, c2_degree>& c2_mat) {
		for (int i = 0; i < c1_degree * c2_degree; i++) {
			int row = i / c2_degree;
			int col = i % c2_degree;

			target[i] = 0;
			for (int j = 0; j < 6; j++) {
				target[i] += c1_mat.v[row][j] * c2_mat.v[j][col];
			}
		}
	}

	template<int c1_degree, int c2_degree>
	static void computeBlockInitialValueDegreed(double* target, DegreedConstraint<c1_degree>* c1, DegreedConstraint<c2_degree>* c2) {
		mthz::NMat<6, c2_degree>* a_c2interaction = nullptr;
		mthz::NMat<6, c2_degree>* b_c2interaction = nullptr;
		if      (c1->a == c2->a) a_c2interaction = &c2->impulse_to_a_velocity;
		else if (c1->a == c2->b) a_c2interaction = &c2->impulse_to_b_velocity;
		else if (c1->b == c2->a) b_c2interaction = &c2->impulse_to_a_velocity; //else because c1 == c2 case handeled elsewhere
		else if (c1->b == c2->b) b_c2interaction = &c2->impulse_to_b_velocity;
		

		if (a_c2interaction != nullptr) matMult(target, c1->a_jacobian, *a_c2interaction);
		else if (b_c2interaction != nullptr) matMult(target, c1->b_jacobian, *b_c2interaction);
		else memset(target, 0x00, c1_degree * c2_degree * sizeof(double));
		//last case occurs when c1 and c2 dont have a direct interaction, but because of gaussian elimination
		//the block will end up having a non-zero value.
	}

	static void computeBlockInitialValue(double* target, Constraint* c1, Constraint* c2, double cfm);

	//note this method computes the dot of a row on the left matrix and a row on the right matrix,
	//NOT a row on the left and a col on the right like you do in normal matrix multiplication.
	//This is because the instances of matrix multiplication where we use this are:
	//Multiplication with a symmetric matrix on the right, so dot with row or col is equivalent
	//Multiplication where we store the right matrix transposed in memory, so we are basically transposing it back with this
	template<int n>
	double dotProd(double* left_mat_row_start, double* right_mat_row_start) {
		double out = 0;
		for (int i = 0; i < n; i++) {
			double d1 = left_mat_row_start[i];
			double d2 = right_mat_row_start[i];
			out += d1 * d2;
		}
		return out;
	}

	static void calculateLowerTriangleBlock(int block_width, int block_height, double* target, double* lower_triangle_source, double* inverse_diagonal_block);

	template<int edge_width>
	static void computeLowerDiagonalBlock(int block_width, int block_height, double* target, double* ldtb_source, double* ldtb_buffer_source);

	void HolonomicSystem::computeInverse(double cfm) {
#ifndef NDEBUG
		if (USE_GAUSS_ELIM_FOR_INVERSE) {
			int row_offset = 0;
			for (int row = 0; row < constraints.size(); row++) {
				int col_offset = 0;
				for (int col = 0; col < constraints.size(); col++) {

					int block_width = constraints[col]->getDegree();
					int block_height = constraints[row]->getDegree();
					std::vector<double> block(block_width * block_height);
					computeBlockInitialValue(block.data(), constraints[row], constraints[col], cfm);

					for (int r = 0; r < block_height; r++) {
						for (int c = 0; c < block_width; c++) {
							debug_inverse[c + col_offset + (r + row_offset) * system_degree] = block[c + r * block_width];
						}
					}

					col_offset += constraints[col]->getDegree();
				}
				row_offset += constraints[row]->getDegree();
			}

			mthz::rowMajorOrderInverse(system_degree, debug_inverse.data(), debug_inverse.data());
			return;
		}
#endif

		//set initial values
		for (int col = 0; col < constraints.size(); col++) {

			for (int row = col; row < constraints.size(); row++) {
				Constraint* c1 = constraints[row];
				Constraint* c2 = constraints[col];
				int loc = getBlockBufferLocation(row, col);
				if (loc == BLOCK_EMPTY) continue;

				double* block_pos = buffer + loc;
				computeBlockInitialValue(block_pos, c1, c2, cfm);
			}
		}

		//debugPrintBuffer("BLOCK_VIEW", false);
		//debugPrintBuffer("COPIED VALUES");

		//compute LDL
		for (int col = 0; col < constraints.size(); col++) {

			//computing inverse of diagonal block
			int block_loc = getBlockBufferLocation(col, col);
			assert(block_loc != BLOCK_EMPTY);
			double* diagonal_block = buffer + block_loc;
			switch (constraints[col]->getDegree()) {
			case 1: mthz::rowMajorOrderInverse<1>(diagonal_block, diagonal_block); break;
			case 2: mthz::rowMajorOrderInverse<2>(diagonal_block, diagonal_block); break;
			case 3: mthz::rowMajorOrderInverse<3>(diagonal_block, diagonal_block); break;
			case 4: mthz::rowMajorOrderInverse<4>(diagonal_block, diagonal_block); break;
			case 5: mthz::rowMajorOrderInverse<5>(diagonal_block, diagonal_block); break;
			case 6: mthz::rowMajorOrderInverse<6>(diagonal_block, diagonal_block); break;
			}

			if (col + 1 == constraints.size()) continue; //no more blocks below or to the right of this.

			//computing lower triagular element for this block
			for (int row = col + 1; row < constraints.size(); row++) { //iterate per block in the column
				int block_buffer_pos = getBlockBufferLocation(row, col);
				if (block_buffer_pos == BLOCK_EMPTY) continue;

				int target_loc = getBlockDiagonalElemBufferLocation(row, col);
				double* diagonal_elem_target = diagonal_elem_buffer + target_loc; //we still need the old value in the lower block, so we are writing to a temporary buffer
				int block_height = constraints[row]->getDegree();
				int block_width = constraints[col]->getDegree();

				assert(target_loc >= 0 && target_loc + block_width * block_height <= diagonal_elem_buffer_capacity);
				calculateLowerTriangleBlock(block_width, block_height, diagonal_elem_target, buffer + block_buffer_pos, diagonal_block);
			}

			//calculate subtraction in the lower diagonal block
			for (int ld_col = col + 1; ld_col < constraints.size(); ld_col++) {
				for (int ld_row = ld_col; ld_row < constraints.size(); ld_row++) {
					int loc = getBlockBufferLocation(ld_row, col);
					if (loc == BLOCK_EMPTY || getBlockBufferLocation(ld_col, col) == BLOCK_EMPTY) continue;

					double* ldtb_source = buffer + loc;
					double* ldtb_buffer_source = diagonal_elem_buffer + getBlockDiagonalElemBufferLocation(ld_col, col);
					int target_loc = getBlockBufferLocation(ld_row, ld_col);
					double* target = buffer + target_loc;
					assert(target_loc != BLOCK_EMPTY);

					switch (constraints[col]->getDegree()) {
					case 1: computeLowerDiagonalBlock<1>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 2: computeLowerDiagonalBlock<2>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 3: computeLowerDiagonalBlock<3>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 4: computeLowerDiagonalBlock<4>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 5: computeLowerDiagonalBlock<5>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 6: computeLowerDiagonalBlock<6>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					}
				}
			}

			//copy diagonal elem buffer back
			int first_non_empty_loc = getFirstNonEmptyBlockBelowDiagonal(col);
			if (first_non_empty_loc == BLOCK_EMPTY) continue;

			assert(first_non_empty_loc >= 0 && first_non_empty_loc < buffer_capacity);
			double* target = buffer + first_non_empty_loc;
			int blocks_total_size = (getBlockBufferLocation(col + 1, col + 1) - first_non_empty_loc) * sizeof(double); //total size in memory of all blocks in the col beneath the diagonal.
			assert(blocks_total_size >= 0 && (blocks_total_size / sizeof(double)) + first_non_empty_loc < buffer_capacity);
			memcpy(target, diagonal_elem_buffer, blocks_total_size);
		}

		//debugPrintBuffer("DONE");
	}

	int HolonomicSystem::getBlockBufferLocation(int block_row, int block_column) {
		int n = constraints.size();

		assert(block_row >= 0 && block_row <= n);
		assert(block_column <= block_row);

		int table_index = block_row - block_column + ((2 * n + 1) * block_column - block_column * block_column) / 2;

		assert(table_index >= 0 && table_index < block_location_table.size());

		return block_location_table[table_index];
	}

	int HolonomicSystem::getFirstNonEmptyBlockBelowDiagonal(int column) {
		for (int row = column + 1; row < constraints.size(); row++) {
			int loc = getBlockBufferLocation(row, column);
			if (loc != BLOCK_EMPTY) return loc;
		}
		return BLOCK_EMPTY;
	}

	int HolonomicSystem::getBlockDiagonalElemBufferLocation(int block_row, int block_column) {
		assert(block_row >= 0 && block_row <= constraints.size());
		assert(block_column < block_row);

		int loc = getBlockBufferLocation(block_row, block_column);
		if (loc == BLOCK_EMPTY) return BLOCK_EMPTY;

		int ret = loc - getFirstNonEmptyBlockBelowDiagonal(block_column);
		assert(ret >= 0 && ret < diagonal_elem_buffer_capacity - constraints[block_row]->getDegree() * constraints[block_column]->getDegree());
		return ret;
	}

	template<int n>
	static void writeTargetDelta(DegreedConstraint<n>* constraint, std::vector<double>* target, int write_index, bool use_psuedo_velocities) {
		assert(write_index >= 0 && write_index + n <= target->size());

		mthz::NVec<n> current_val = use_psuedo_velocities? constraint->getConstraintValue(*constraint->a_psuedo_velocity_change, *constraint->b_psuedo_velocity_change)
			                                             : constraint->getConstraintValue(*constraint->a_velocity_change, *constraint->b_velocity_change);
		mthz::NVec<n> target_val = use_psuedo_velocities ? constraint->psuedo_target_val : constraint->target_val;

		mthz::NVec<n> delta = target_val - current_val;
		for (int j = 0; j < n; j++) {
			target->at(write_index + j) = delta.v[j];
		}
	}

	template<int block_width, int block_height>
	static void multMatWithVecDegreed(std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source) {
		double* vec_source = source_vector.data() + source_index;
		for (int i = 0; i < block_height; i++) {
			target->at(write_index + i) = dotProd<block_width>(matrix_source + i * block_width, vec_source);
		}
	}

	static void multMatWithVec(int block_width, int block_height, std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source) {
		switch (block_width) {
		case 1:
			switch (block_height) {
			case 1: multMatWithVecDegreed<1, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<1, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<1, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<1, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<1, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<1, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 2:
			switch (block_height) {
			case 1: multMatWithVecDegreed<2, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<2, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<2, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<2, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<2, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<2, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 3:
			switch (block_height) {
			case 1: multMatWithVecDegreed<3, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<3, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<3, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<3, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<3, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<3, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 4:
			switch (block_height) {
			case 1: multMatWithVecDegreed<4, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<4, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<4, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<4, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<4, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<4, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 5:
			switch (block_height) {
			case 1: multMatWithVecDegreed<5, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<5, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<5, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<5, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<5, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<5, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 6:
			switch (block_height) {
			case 1: multMatWithVecDegreed<6, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<6, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<6, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<6, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<6, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<6, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		}
	}

	template<int block_width, int block_height>
	static void multMatTransposedWithVecDegreed(std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source) {
		double* vec_source = source_vector.data() + source_index;
		for (int i = 0; i < block_width; i++) {
			double dotprod = 0;
			for (int j = 0; j < block_height; j++) {
				double d1 = source_vector[source_index + j];
				double d2 = matrix_source[i + block_width * j];
				dotprod += d1 * d2;
			}
			target->at(write_index + i) += dotprod;
		}
	}

	static void multMatTransposedWithVec(int block_width, int block_height, std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source) {
		switch (block_width) {
		case 1:
			switch (block_height) {
			case 1: multMatTransposedWithVecDegreed<1, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatTransposedWithVecDegreed<1, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatTransposedWithVecDegreed<1, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatTransposedWithVecDegreed<1, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatTransposedWithVecDegreed<1, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatTransposedWithVecDegreed<1, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 2:
			switch (block_height) {
			case 1: multMatTransposedWithVecDegreed<2, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatTransposedWithVecDegreed<2, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatTransposedWithVecDegreed<2, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatTransposedWithVecDegreed<2, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatTransposedWithVecDegreed<2, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatTransposedWithVecDegreed<2, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 3:
			switch (block_height) {
			case 1: multMatTransposedWithVecDegreed<3, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatTransposedWithVecDegreed<3, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatTransposedWithVecDegreed<3, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatTransposedWithVecDegreed<3, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatTransposedWithVecDegreed<3, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatTransposedWithVecDegreed<3, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 4:
			switch (block_height) {
			case 1: multMatTransposedWithVecDegreed<4, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatTransposedWithVecDegreed<4, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatTransposedWithVecDegreed<4, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatTransposedWithVecDegreed<4, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatTransposedWithVecDegreed<4, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatTransposedWithVecDegreed<4, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 5:
			switch (block_height) {
			case 1: multMatTransposedWithVecDegreed<5, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatTransposedWithVecDegreed<5, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatTransposedWithVecDegreed<5, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatTransposedWithVecDegreed<5, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatTransposedWithVecDegreed<5, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatTransposedWithVecDegreed<5, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 6:
			switch (block_height) {
			case 1: multMatTransposedWithVecDegreed<6, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatTransposedWithVecDegreed<6, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatTransposedWithVecDegreed<6, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatTransposedWithVecDegreed<6, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatTransposedWithVecDegreed<6, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatTransposedWithVecDegreed<6, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		}
	}

	static void computeBlockInitialValue(double* block_pos, Constraint* c1, Constraint* c2, double cfm) {
		if (c1 == c2) {
			double* source = nullptr;
			int n = c1->getDegree();
			switch (n) {
			case 1: source = (double*)((DegreedConstraint<1>*)c1)->impulse_to_value.v; break;
			case 2: source = (double*)((DegreedConstraint<2>*)c1)->impulse_to_value.v; break;
			case 3: source = (double*)((DegreedConstraint<3>*)c1)->impulse_to_value.v; break;
			case 4: source = (double*)((DegreedConstraint<4>*)c1)->impulse_to_value.v; break;
			case 5: source = (double*)((DegreedConstraint<5>*)c1)->impulse_to_value.v; break;
			case 6: source = (double*)((DegreedConstraint<6>*)c1)->impulse_to_value.v; break;
			}
			memcpy(block_pos, source, c1->getDegree() * c2->getDegree() * sizeof(double));

			
			for (int i = 0; i < n; i++) {
				block_pos[i + n * i] *= (1 + cfm); //add cfm along the diagonal
			}
			return;
		}

		//lol
		switch (c1->getDegree()) {
		case 1:
		{
			DegreedConstraint<1>* d1 = (DegreedConstraint<1>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 2:
		{
			DegreedConstraint<2>* d1 = (DegreedConstraint<2>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 3:
		{
			DegreedConstraint<3>* d1 = (DegreedConstraint<3>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 4:
		{
			DegreedConstraint<4>* d1 = (DegreedConstraint<4>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 5:
		{
			DegreedConstraint<5>* d1 = (DegreedConstraint<5>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 6:
		{
			DegreedConstraint<6>* d1 = (DegreedConstraint<6>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		}
	}

	template<int block_width, int block_height>
	static void calculateLowerTriangleBlockDegreed(double* target, double* lower_triangle_source, double* inverse_diagonal_block) {
		for (int row = 0; row < block_height; row++) { 
			for (int col = 0; col < block_width; col++) { 

				double dot_prod = dotProd<block_width>(lower_triangle_source + block_width * row, inverse_diagonal_block + block_width * col);
				target[col + block_width * row] = dot_prod;
			}
		}
	}

	static void calculateLowerTriangleBlock(int block_width, int block_height, double* target, double* lower_triangle_source, double* inverse_diagonal_block) {
		switch (block_width) {
		case 1:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<1, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<1, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<1, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<1, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<1, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<1, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 2:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<2, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<2, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<2, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<2, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<2, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<2, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 3:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<3, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<3, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<3, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<3, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<3, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<3, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 4:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<4, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<4, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<4, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<4, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<4, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<4, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 5:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<5, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<5, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<5, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<5, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<5, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<5, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 6:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<6, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<6, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<6, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<6, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<6, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<6, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		}
	}

	template<int block_width, int block_height, int edge_width>
	static void computeLowerDiagonalBlockDegreed(double* target, double* ldtb_source, double* ldtb_buffer_source) {
		for (int row = 0; row < block_height; row++) {
			for (int col = 0; col < block_width; col++) {
				target[row * block_width + col] -= dotProd<edge_width>(ldtb_source + row * edge_width, ldtb_buffer_source + col * edge_width);
			}
		}
	}

	template<int edge_width>
	static void computeLowerDiagonalBlock(int block_width, int block_height, double* target, double* ldtb_source, double* ldtb_buffer_source) {
		switch (block_width) {
		case 1:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<1, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<1, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<1, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<1, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<1, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<1, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 2:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<2, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<2, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<2, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<2, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<2, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<2, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 3:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<3, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<3, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<3, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<3, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<3, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<3, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 4:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<4, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<4, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<4, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<4, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<4, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<4, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 5:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<5, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<5, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<5, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<5, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<5, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<5, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 6:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<6, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<6, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<6, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<6, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<6, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<6, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		}
	}

	void HolonomicSystem::debugPrintBuffer(std::string name, bool print_val) {
		printf("\n===========%s=========\n", name.c_str());
		if (print_val) {
			for (int row = 0; row < constraints.size(); row++) {
				for (int sub_row = 0; sub_row < constraints[row]->getDegree(); sub_row++) {
					for (int col = 0; col < constraints.size(); col++) {
						for (int sub_col = 0; sub_col < constraints[col]->getDegree(); sub_col++) {

							if (col > row) {
								printf("%8c", 'U');
							}
							else if (getBlockBufferLocation(row, col) == BLOCK_EMPTY) {
								printf("%8c", '.');
							}
							else {
								int indx = getBlockBufferLocation(row, col) + sub_col + constraints[col]->getDegree() * sub_row;
								printf("%8.3f", buffer[indx]);
							}

						}
					}
					printf("\n");
				}
			}
		}
		else {
			for (int row = 0; row < constraints.size(); row++) {
				for (int col = 0; col < constraints.size(); col++) {
					if (col > row) {
						printf("U  ");
					}
					else if (getBlockBufferLocation(row, col) == BLOCK_EMPTY) {
						printf(".  ");
					}
					else {
						printf("R  ");
					}
				}
				printf("\n");
			}
		}
	}
};