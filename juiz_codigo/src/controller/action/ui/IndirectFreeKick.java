package controller.action.ui;

import common.Log;
import controller.action.ActionBoard;
import controller.action.ActionType;
import controller.action.GCAction;
import data.Rules;
import data.communication.TeamInfo;
import data.states.AdvancedData;
import data.values.GameStates;
import data.values.SecondaryGameStates;

/**
 * @author Robert Kessler
 * 
 * This action performs the switch to the Secondary Game State: Free Kick
 * It stops the global clock and prepares the secondary clock to run down to zero
 * When the action is resolved then - the normal clock continues and the previous state
 * is taken again. Based on which side the action is executed for the FreeKick Mode in the
 * Secondary State Object is updated
 */
public class IndirectFreeKick extends GCAction
{
    /** On which side (0:left, 1:right) */
    private int side;

    public IndirectFreeKick(int side)
    {
        super(ActionType.UI);
        this.side = side;
    }

    @Override
    public void perform(AdvancedData data)
    {
        boolean isInDirectfreeKick = data.secGameState == SecondaryGameStates.INDIRECT_FREEKICK;

        // Multiple possibilities

        // If we are not in DirectFreeKick we can switch to the first state of it meaning
        // secondaryGameState = DIRECT_FREEKICK and secondaryGameStateInfo = (teamnumber, state)
        if (!isInDirectfreeKick) {
            data.previousSecGameState = data.secGameState;
            data.secGameState = SecondaryGameStates.INDIRECT_FREEKICK;
            data.secGameStateInfo.setFreeKickData(data.team[side].teamNumber, (byte) 1);


            Log.setNextMessage("IndirectFreeKick " + data.team[side].teamColor.toString());
            ActionBoard.clockPause.perform(data);
        } else {
            // Otherwise we need to check in which sub mode we are - and move one forward if not at 1
            byte team = data.secGameStateInfo.toByteArray()[0];
            byte subMode = data.secGameStateInfo.toByteArray()[1];

            if (subMode == 1){
                data.secGameStateInfo.setFreeKickData(team, (byte) 0);
                data.gameClock.setSecondaryClock(Rules.league.free_kick_preparation_time);
            } else {
                data.secGameState = data.previousSecGameState;
                data.previousSecGameState = SecondaryGameStates.INDIRECT_FREEKICK;
                data.secGameStateInfo.reset();
                data.gameClock.clearSecondaryClock();
                Log.setNextMessage("End IndirectFreeKick " + data.team[side].teamColor.toString());
                data.gameClock.addExtraClock("Indirect Free Kick", 10);
                ActionBoard.clockPause.perform(data);
            }
        }
    }

    @Override
    public boolean isLegal(AdvancedData data)
    {
        boolean ifInDirectFreeKickItsMe = true;

        if (data.secGameState == SecondaryGameStates.INDIRECT_FREEKICK){
            ifInDirectFreeKickItsMe = data.secGameStateInfo.toByteArray()[0] == data.team[side].teamNumber;
        }

        return data.testmode || data.gameState == GameStates.PLAYING
                && ifInDirectFreeKickItsMe
                && data.secGameState != SecondaryGameStates.PENALTYKICK
                && data.secGameState != SecondaryGameStates.PENALTYSHOOT
                && data.secGameState != SecondaryGameStates.DIRECT_FREEKICK;
    }
}