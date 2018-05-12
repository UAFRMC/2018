
const char *state_to_string(robot_state_t state)
{
	if (state<0 || state>=state_last) return "unknown_invalid";
	const static char *table[state_last+1]={
	"STOP", ///< EMERGENCY STOP (no motion)
	"drive", ///< normal manual driving
	"backend_driver", ///< drive from backend UI

	"autonomy",
	"raise",
	"find_camera",
	"align_turnout",
	"align_drive", ///< drive to start area
	"align_turnin",
	"align_back",

	"drive_to_mine", ///< autonomous: drive to mining area

	/* Semiauto mine mode entry point: */
	"mine_lower", ///< mining mode: lowering head", driving forward
	"mine_stall", ///< mining mode: raising head (after stall)
	"mine", // actually mine
	"mine_raise", ///< existing mining mode: raise bucket

	"drive_to_dump", ///< drive back to bin

	/* Semiauto dump mode entry points: */
	"dump_contact", ///< final dock-and-dump mode: drive to contact bin
	"dump_raise", ///< raising bucket
	"dump_pull", ///< roll forward
	"dump_rattle", ///< rattle mode to empty bucket
	"dump_push", ///< push hook back
	"dump_lower", ///< lowering bucket (after dump)

	/* Stow mode (shutdown sequence) */
	"stow", // begin stowing: raise bucket
	"stow_clean", // clean out bucket
	"stowed", // finished stowing (wait forever)

	"last"
	};

	return table[state];
}


