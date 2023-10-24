#pragma once

namespace phyz {
	enum CFM_MODE { USE_GLOBAL, USE_CUSTOM };
	struct CFM {
		CFM_MODE mode;
		double custom_cfm_value;

		double getCFMValue(double global_cfm) {
			return (mode == USE_GLOBAL) ? global_cfm : custom_cfm_value;
		}
	};

	CFM averageCFM(double global_cfm, CFM a, CFM b);
}