using UnityEngine;

namespace KartGame.KartSystems {

    public class KeyboardInput : BaseInput
    {
        public string TurnInputName = "Horizontal";
        public string AccelerateButtonName = "Accelerate";
        public string BrakeButtonName = "Brake";

        public override InputData GenerateInput() {
            return new InputData
            {
                Accelerate = Input.GetButton(AccelerateButtonName),
                Brake = Input.GetButton(BrakeButtonName),
                TurnInput = Input.GetAxis("Horizontal")
            };
            // return new InputData
            // {
            //     Accelerate = (Input.GetAxis(AccelerateButtonName)>0.2f),
            //     Brake = (Input.GetAxis(BrakeButtonName)>0.2f),
            //     TurnInput = Input.GetAxis("Horizontal")
            // };
        }
    }
}
