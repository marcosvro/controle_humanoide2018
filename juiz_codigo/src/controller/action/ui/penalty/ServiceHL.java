package controller.action.ui.penalty;

import common.Log;
import data.states.AdvancedData;
import data.PlayerInfo;
import data.values.Penalties;

/**
 *
 * @author Daniel Seifert
 */
public class ServiceHL extends Penalty
{
    /**
     * Performs this action`s penalty on a selected player.
     *
     * @param data      The current data to work on.
     * @param player    The player to penalise.
     * @param side      The side the player is playing on (0:left, 1:right).
     * @param number    The player`s number, beginning with 0!
     */
    @Override
    public void performOn(AdvancedData data, PlayerInfo player, int side, int number)
    {
        if (player.penalty == Penalties.NONE) {
            data.whenPenalized[side][number] = data.getTime();
            player.penalty = Penalties.HL_SERVICE;
            handleRepeatedPenalty(data, player, side, number);
            Log.state(data, "Request for Service " + data.team[side].teamColor + " " + (number+1));
        } else {
            player.penalty = Penalties.HL_SERVICE;
            handleRepeatedPenalty(data, player, side, number);
            Log.state(data, "Additional Request for Service " + data.team[side].teamColor + " " + (number+1));
        }
    }

    @Override
    public boolean isLegal(AdvancedData data)
    {
        return true;
    }
}
