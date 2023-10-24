#include "CFM.h"

namespace phyz {
	CFM averageCFM(double global_cfm, CFM a, CFM b) {
		CFM out;

		if (a.mode == USE_GLOBAL) a.custom_cfm_value = global_cfm;
		if (b.mode == USE_GLOBAL) b.custom_cfm_value = global_cfm;
		out.custom_cfm_value = (a.custom_cfm_value + b.custom_cfm_value) / 2.0;

		if (a.mode == USE_GLOBAL && b.mode == USE_GLOBAL)	out.mode = USE_GLOBAL;
		else												out.mode = USE_CUSTOM;

		return out;
	}
}