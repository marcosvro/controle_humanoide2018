package controller.action.ui;

import common.Log;
import controller.action.ActionBoard;
import controller.action.ActionType;
import controller.action.GCAction;
import data.Rules;
import data.states.AdvancedData;
import data.values.GameStates;
import data.values.SecondaryGameStates;

/**
 * @author Michel Bartsch
 * 
 * This action means that a timeOut is to be taken or ending.
 */
public class PenaltyKick extends GCAction
{
    /** On which side (0:left, 1:right) */
    private int side;

    /**
     * Creates a new TimeOut action.
     * Look at the ActionBoard before using this.
     *
     * @param side      On which side (0:left, 1:right)
     */
    public PenaltyKick(int side)
    {
        super(ActionType.UI);
        this.side = side;
    }

    /**
     * Performs this action to manipulate the data (model).
     * 
     * @param data      The current data to work on.
     */
    @Override
    public void perform(AdvancedData data)
    {
        boolean isInPenaltyKick = data.secGameState == SecondaryGameStates.PENALTYKICK;

        if (!isInPenaltyKick) {
            data.previousSecGameState = data.secGameState;
            data.secGameState = SecondaryGameStates.PENALTYKICK;
            data.secGameStateInfo.switchToPenaltyKick(data.team[side].teamNumber, (byte) 1);

            Log.setNextMessage("PenaltyKick " + data.team[side].teamColor);
            ActionBoard.clockPause.perform(data);
        } else {
            byte team = data.secGameStateInfo.toByteArray()[0];
            byte subMode = data.secGameStateInfo.toByteArray()[1];

            if (subMode == 1){
                data.secGameStateInfo.setFreeKickData(team, (byte) 0);
                data.gameClock.setSecondaryClock(Rules.league.penalty_kick_preparation_time);
            } else {
                data.secGameState = data.previousSecGameState;
                data.previousSecGameState = SecondaryGameStates.PENALTYKICK;
                data.secGameStateInfo.reset();
                data.gameClock.clearSecondaryClock();

                Log.setNextMessage("End PenaltyKick " + data.team[side].teamColor);
                ActionBoard.clockPause.perform(data);
//            data.gameClock.setSecondaryClock(10);
            }
        }
    }
    
    /**
     * Checks if this action is legal with the given data (model).
     * Illegal actions are not performed by the EventHandler.
     * 
     * @param data      The current data to check with.
     */
    @Override
    public boolean isLegal(AdvancedData data)
    {
        boolean ifInPenaltyKick = true;

        if (data.secGameState == SecondaryGameStates.PENALTYKICK){
            ifInPenaltyKick = data.secGameStateInfo.toByteArray()[0] == data.team[side].teamNumber;
        }

      return data.testmode || data.gameState == GameStates.PLAYING
              && ifInPenaltyKick
              && data.secGameState != SecondaryGameStates.DIRECT_FREEKICK
              && data.secGameState != SecondaryGameStates.PENALTYSHOOT
              && data.secGameState != SecondaryGameStates.INDIRECT_FREEKICK;
    }
}