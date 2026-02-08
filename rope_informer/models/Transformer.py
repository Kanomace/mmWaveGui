import torch
import torch.nn as nn
import torch.nn.functional as F
from models.Transformer_EncDec import Decoder, DecoderLayer, Encoder, EncoderLayer, ConvLayer
from models.SelfAttention_Family import FullAttention, AttentionLayer
from models.Embed1 import DataEmbedding
import numpy as np


class Model(nn.Module):
    """
    Vanilla Transformer
    with O(L^2) complexity
    Paper link: https://proceedings.neurips.cc/paper/2017/file/3f5ee243547dee91fbd053c1c4a845aa-Paper.pdf
    """

    def __init__(self, enc_in, dec_in, c_out, seq_len, label_len, pred_len, 
                factor=5, d_model=512, n_heads=8, e_layers=3, d_layers=2, d_ff=512, 
                dropout=0.0, attn='prob', embed='fixed', freq='h', activation='gelu', 
                output_attention = False, distil=False, mix=True, has_rope = False, num_class = 10,
                device=torch.device('cuda:0')):
        super(Model, self).__init__()
        self.task_name = "long_term_forecast"
        self.pred_len = pred_len
        # self.output_attention = configs.output_attention
        # Embedding
        self.enc_embedding = DataEmbedding(enc_in, d_model, d_model, freq,
                                           dropout)
        # Encoder
        self.encoder = Encoder(
            [
                EncoderLayer(
                    AttentionLayer(
                        FullAttention(False, factor, attention_dropout=dropout,
                                      output_attention=False), d_model, n_heads),
                    d_model,
                    d_ff,
                    dropout=dropout,
                    activation=activation
                ) for l in range(e_layers)
            ],
            norm_layer=torch.nn.LayerNorm(d_model)
        )
        # Decoder
        if self.task_name == 'long_term_forecast' or self.task_name == 'short_term_forecast':
            self.dec_embedding = DataEmbedding(dec_in, d_model, embed, freq,
                                               dropout)
            self.decoder = Decoder(
                [
                    DecoderLayer(
                        AttentionLayer(
                            FullAttention(True, factor, attention_dropout=dropout,
                                          output_attention=False),
                            d_model, n_heads),
                        AttentionLayer(
                            FullAttention(False, factor, attention_dropout=dropout,
                                          output_attention=False),
                            d_model, n_heads),
                            d_model,
                            d_ff,
                        dropout=dropout,
                        activation=activation,
                    )
                    for l in range(d_layers)
                ],
                norm_layer=torch.nn.LayerNorm(d_model),
                projection=nn.Linear(d_model, c_out, bias=True)
            )
        self.act = F.gelu
        self.dropout = nn.Dropout(dropout)
        self.projection = nn.Linear(d_model * seq_len, num_class)

    def forward(self, x_enc, x_dec = None, 
                enc_self_mask=None, dec_self_mask=None, x_mark_enc = None, x_mark_dec = None, dec_enc_mask=None):

        # enc
        x_enc = x_enc.unsqueeze(-1)  # batch x seq x t
        enc_out = self.enc_embedding(x_enc, None)
        enc_out, attns = self.encoder(enc_out, attn_mask=None)

        # Output
        output = self.act(enc_out)  
        output = self.dropout(output)
        # output = output * x_mark_enc.unsqueeze(-1)  
        output = output.reshape(output.shape[0], -1)  
        output = self.projection(output)  # (batch_size, num_classes)
        return output

    # def forecast(self, x_enc,  x_dec, x_mark_enc = None, x_mark_dec = None):
    #     # Embedding
    #     enc_out = self.enc_embedding(x_enc, x_mark_enc)
    #     enc_out, attns = self.encoder(enc_out, attn_mask=None)

    #     dec_out = self.dec_embedding(x_dec, x_mark_dec)
    #     dec_out = self.decoder(dec_out, enc_out, x_mask=None, cross_mask=None)
    #     return dec_out
    
    # def forward(self, x_enc,  x_dec, x_mark_enc = None, x_mark_dec = None, mask=None):
    #     if self.task_name == 'long_term_forecast' or self.task_name == 'short_term_forecast':
    #         dec_out = self.forecast(x_enc,  x_dec,x_mark_enc, x_mark_dec)
    #         return dec_out[:, -self.pred_len:, :]  # [B, L, D]
    #     return None
