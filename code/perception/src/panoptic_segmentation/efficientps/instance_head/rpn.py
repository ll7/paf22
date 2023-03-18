from inplace_abn import InPlaceABN
# detectron 2 dependencies
from detectron2.modeling.proposal_generator.rpn import StandardRPNHead
from detectron2.structures import ImageList
from detectron2.modeling.proposal_generator import (
    RPN,
    RPN_HEAD_REGISTRY,
    PROPOSAL_GENERATOR_REGISTRY
)

try:
    from panoptic_segmentation \
        .efficientps.utils.depthwise_separable_conv import \
        DepthwiseSeparableConv
except ImportError:
    from efficientps.utils.depthwise_separable_conv import \
        DepthwiseSeparableConv

print("HELLO WORLD")


@PROPOSAL_GENERATOR_REGISTRY.register()
class RPNCustom(RPN):

    def forward(
        self,
        features,
        gt_instances
        # images: ImageList,
        # features: Dict[str, torch.Tensor],
        # gt_instances: Optional[List[Instances]] = None,
    ):
        """
    Create the different input needed for detectron2 forward method
    """
        # Create image a ImageList instance only use for the image_size
        batch_size = features['P_4'].shape[0]
        image_size = (features['P_4'].shape[2] * 4,
                      features['P_4'].shape[3] * 4)
        images = ImageList(None, [image_size for i in range(batch_size)])
        return super().forward(images, features, gt_instances)


@RPN_HEAD_REGISTRY.register()
class DepthwiseSepRPNHead(StandardRPNHead):

    def __init__(self, cfg, input_shape):
        super().__init__(cfg, input_shape)
        # Modify version of the convolution and Inplace batchNorm
        in_channels = input_shape[0].channels
        self.conv = DepthwiseSeparableConv(in_channels,
                                           in_channels,
                                           kernel_size=3,
                                           padding=1)
        self.iabn = InPlaceABN(in_channels)

    def forward(self, features):
        """
        Same forward as in detectron with iabn instead of Relu
        Args:
            features (list[Tensor]): list of feature maps

        Returns:
            list[Tensor]: A list of L elements.
                Element i is a tensor of shape (N, A, Hi, Wi) representing
                the predicted objectness logits for all anchors.
                A is the number of cell anchors.
            list[Tensor]: A list of L elements. Element i is a tensor of shape
                (N, A*box_dim, Hi, Wi) representing the predicted "deltas"
                used to transform anchors to proposals.
        """
        pred_objectness_logits = []
        pred_anchor_deltas = []
        for x in features:
            t = self.iabn(self.conv(x))
            pred_objectness_logits.append(self.objectness_logits(t))
            pred_anchor_deltas.append(self.anchor_deltas(t))
        return pred_objectness_logits, pred_anchor_deltas
