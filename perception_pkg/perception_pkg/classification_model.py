import torch
import torch.nn as nn

class PointNetEncoderXYZRGB(nn.Module):
    """Encoder for Pointcloud
    """

    def __init__(self,
                 in_channels: int,
                 out_channels: int=1024,
                 use_layernorm: bool=False,
                 final_norm: str='none',
                 use_projection: bool=True,
                 **kwargs
                 ):
        """_summary_

        Args:
            in_channels (int): feature size of input (3 or 6)
            input_transform (bool, optional): whether to use transformation for coordinates. Defaults to True.
            feature_transform (bool, optional): whether to use transformation for features. Defaults to True.
            is_seg (bool, optional): for segmentation or classification. Defaults to False.
        """
        super().__init__()
        block_channel = [32, 256]
        print("pointnet use_layernorm: {}".format(use_layernorm))
        print("pointnet use_final_norm: {}".format(final_norm))
        
        self.mlp = nn.Sequential(
            nn.Linear(in_channels, block_channel[0]),
            nn.LayerNorm(block_channel[0]) if use_layernorm else nn.Identity(),
            nn.ReLU(),
            nn.Linear(block_channel[0], block_channel[1]),
            # nn.LayerNorm(block_channel[1]) if use_layernorm else nn.Identity(),
            # nn.ReLU(),
            # nn.Linear(block_channel[1], block_channel[2]),
            # nn.LayerNorm(block_channel[2]) if use_layernorm else nn.Identity(),
            # nn.ReLU(),
            # nn.Linear(block_channel[2], block_channel[3]),
        )
        
       
        if final_norm == 'layernorm':
            self.final_projection = nn.Sequential(
                nn.Linear(block_channel[-1], out_channels),
                nn.LayerNorm(out_channels)
            )
        elif final_norm == 'none':
            self.final_projection = nn.Linear(block_channel[-1], out_channels)
        else:
            raise NotImplementedError(f"final_norm: {final_norm}")
         
    def forward(self, x):
        x = self.mlp(x)
        x = torch.max(x, 1)[0]
        x = self.final_projection(x)
        return x


class PointNetClassifier(torch.nn.Module):
    def __init__(self, encoder: PointNetEncoderXYZRGB, num_classes: int):
        super().__init__()
        self.encoder = encoder
        self.classifier = nn.Sequential(
            nn.Linear(self.encoder.final_projection.out_features, 256),
            nn.ReLU(),
            # torch.nn.Dropout(0.5),
            nn.Linear(256, 32),
            nn.ReLU(),
            # torch.nn.Dropout(0.5),
            nn.Linear(32, num_classes),
        )

    def forward(self, x):
        x = self.encoder(x)
        x = self.classifier(x)
        return x