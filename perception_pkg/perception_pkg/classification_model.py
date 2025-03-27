import torch
import torch.nn as nn
import torch.nn.functional as F

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


def knn(x, k):
    inner = -2 * torch.matmul(x.transpose(2, 1).contiguous(), x)
    xx = torch.sum(x**2, dim=1, keepdim=True)
    pairwise_distance = -xx - inner - xx.transpose(2, 1).contiguous()

    idx = pairwise_distance.topk(k=k, dim=-1)[1]  # (batch_size, num_points, k)
    return idx



def get_graph_feature(x, k=20):
    # x = x.squeeze()
    idx = knn(x, k=k)  # (batch_size, num_points, k)
    batch_size, num_points, _ = idx.size()
    device = x.device

    idx_base = torch.arange(0, batch_size, device=device).view(-1, 1, 1) * num_points

    idx = idx + idx_base

    idx = idx.view(-1)

    _, num_dims, _ = x.size()

    x = x.transpose(
        2, 1
    ).contiguous()  # (batch_size, num_points, num_dims)  -> (batch_size*num_points, num_dims) #   batch_size * num_points * k + range(0, batch_size*num_points)
    feature = x.view(batch_size * num_points, -1)[idx, :]
    feature = feature.view(batch_size, num_points, k, num_dims)
    x = x.view(batch_size, num_points, 1, num_dims).repeat(1, 1, k, 1)

    feature = torch.cat((feature, x), dim=3).permute(0, 3, 1, 2)

    return feature


class DGCNN(nn.Module):
    def __init__(self, input_dims=3, emb_dims=512):
        super(DGCNN, self).__init__()
        self.conv1 = nn.Conv2d(input_dims*2, 64, kernel_size=1, bias=False)
        self.conv2 = nn.Conv2d(64, 64, kernel_size=1, bias=False)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=1, bias=False)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=1, bias=False)
        self.conv5 = nn.Conv2d(512, emb_dims, kernel_size=1, bias=False)
        self.bn1 = nn.BatchNorm2d(64)
        self.bn2 = nn.BatchNorm2d(64)
        self.bn3 = nn.BatchNorm2d(128)
        self.bn4 = nn.BatchNorm2d(256)
        self.bn5 = nn.BatchNorm2d(emb_dims)

    def forward(self, x):
        batch_size, num_dims, num_points = x.size()
        x = get_graph_feature(x)
        x = F.relu(self.bn1(self.conv1(x)))
        x1 = x.max(dim=-1, keepdim=True)[0]

        x = F.relu(self.bn2(self.conv2(x)))
        x2 = x.max(dim=-1, keepdim=True)[0]

        x = F.relu(self.bn3(self.conv3(x)))
        x3 = x.max(dim=-1, keepdim=True)[0]

        x = F.relu(self.bn4(self.conv4(x)))
        x4 = x.max(dim=-1, keepdim=True)[0]

        x = torch.cat((x1, x2, x3, x4), dim=1)

        x = F.relu(self.bn5(self.conv5(x))).view(batch_size, -1, num_points)
        return x
    
class LinearDecoder(nn.Module):
    def __init__(self, emb_dims, num_classes):
        super(LinearDecoder, self).__init__()
        self.linear1 = nn.Linear(emb_dims, num_classes)
        # self.linear2 = nn.Linear(emb_dims, emb_dims // 2)
        # self.linear2 = nn.Linear(emb_dims // 2, num_classes)

    def forward(self, x):
        # x = F.relu(self.linear1(x))
        # x = self.linear1(x)
        logits = self.linear1(x)
        return F.softmax(logits, dim=-1)
    
class DGCNNClassifier(nn.Module):
    """
    DGCNN model with a classification head
    """
    def __init__(self, input_dims=3, emb_dims=512, num_classes=5):
        super(DGCNNClassifier, self).__init__()
        self.dgcnn = DGCNN(input_dims=input_dims, emb_dims=emb_dims)
        self.classifier = LinearDecoder(emb_dims, num_classes)
        
    def forward(self, x):
        # Get features from DGCNN
        features = self.dgcnn(x)
        # Global max pooling
        global_features = torch.max(features, dim=2)[0]
        # Classification
        logits = self.classifier(global_features)
        return logits