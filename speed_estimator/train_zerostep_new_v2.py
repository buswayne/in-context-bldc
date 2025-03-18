from pathlib import Path
import time
import torch
import numpy as np
import math
import gc
from functools import partial
from torch.utils.data import DataLoader
from transformer_zerostep_new_v2 import GPTConfig, GPT, warmup_cosine_lr
import argparse
import warnings
import wandb
import torch.nn as nn
import pandas as pd
import copy

### quick param selection
### ckpt_zerostep_sim_matlab_50pct_mix_real_val_noise_h50

checkpoint_name_to_save = "ckpt_50pct_recursive_h50_real_val_speed_correction_v2"
checkpoint_name_to_open = "ckpt_50pct_recursive_h50_real_val_speed_correction_v2"
mode = "resume"  # resume / scratch / pretrained

sequence_length = 50
# noise_std_value = 200
batch_size_ = 64
max_iteration_number = 20_000
learning_rate_value = 1e-5

layers_number = 8 #8
heads_number = 4 #4
embd_number = 16 #16



# folder_path_training = ['../data/simulated/50_percent_longer_steps', '../data/simulated/50_percent_shorter_steps']
folder_path_training = ['../../../in-context-bldc-data/simulated/50_percent_add_with_alfa_beta_speed_corrected']
# folder_path_training = ['../../../in-context-bldc-data/simulated/50_percent_with_alfa_beta_speed_corrected','../../../in-context-bldc-data/simulated/50_percent_add_with_alfa_beta_speed_corrected']
# folder_path_training = ['../data/CL_experiments_double_sensor/train/inertia13_ki-0.0061-kp-11.8427']
# folder_path_training = ['../../../in-context-bldc-data/simulated/50_percent_with_alfa_beta']
# folder_path_val = ['../data/CL_experiments/train/inertia13_ki-0.0061-kp-11.8427','../data/CL_experiments/test/inertia07_ki-0.0061-kp-11.8427','../data/CL_experiments/test/inertia04_ki-0.0061-kp-11.8427']
folder_path_val = ['../data/CL_experiments_double_sensor/train/inertia13_ki-0.0061-kp-11.8427']
# folder_path_val = folder_path_training


weird_stuff = True
if weird_stuff:
    from dataset_new_v2_alt import Dataset, load_dataframes_from_folder
else:
    from dataset_new_v2 import Dataset, load_dataframes_from_folder
# from dataset_new_v2 import Dataset, load_dataframes_from_folder




# Disable all user warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Re-enable user warnings
# warnings.filterwarnings("default")

# # # start a new wandb run to track this script
wandb.init(
    # set the wandb project where this run will be logged
    project="in-context bldc estimator",
    name=checkpoint_name_to_save
)


# def train(model, dataloader, criterion, optimizer, device):
#     torch.autograd.set_detect_anomaly(True)
#     model.train()
#     running_loss = 0.0
#     for batch in dataloader:
#         batch_u, batch_y = batch
#         batch_u, batch_y = batch_u.to(device), batch_y.to(device)

#         optimizer.zero_grad()

#         batch_y_pred = torch.zeros_like(batch_y)
        
#         # create a copy of batch_u to work with, then overwrite the real velocity (symbolic, may not be needed for the code)
#         batch_u_copy = batch_u.clone().detach()
#         batch_u_copy[:,:,4] = 0

#         # simulate step by step
#         last_predictions = torch.zeros(batch_u_copy.shape[0], device=device)

#         for t in range(batch_u_copy.shape[1]):
#             batch_u_step = batch_u_copy.clone()
#             batch_u_step[:,t,4] = last_predictions
#             batch_u_tmp = batch_u_step[:,:t+1,:]
#             #update last predictions
#             last_predictions = model(batch_u_tmp)[:,-1,:].view(-1)
#             batch_y_pred[:,t,0] = last_predictions

#         loss = criterion(batch_y, batch_y_pred)

#         loss.backward()
#         optimizer.step()

#         running_loss += loss.item()

#         for name, param in model.named_parameters():
#             if param.grad is None:
#                 print(f"No gradient computed for {name}")

#     return running_loss / len(dataloader)


def train(model, dataloader, criterion, optimizer, device):
    torch.autograd.set_detect_anomaly(True)
    model.train()
    running_loss = 0.0
    
    for batch in dataloader:
        batch_u, batch_y = batch
        batch_u, batch_y = batch_u.to(device), batch_y.to(device)

        optimizer.zero_grad()  # Clear previous gradients

        # Create a copy of batch_u to work with, and set the velocity column (index 4) to zero
        batch_u_copy = batch_u.clone()
        batch_u_copy[:,:,4] = 0  # No need to detach() because we want gradients to propagate

        # Store predictions
        last_predictions = torch.zeros(batch_u_copy.shape[0], device=device, requires_grad=True)
        batch_y_pred_list = []  # Use a list to accumulate outputs

        # Simulate step by step
        for t in range(batch_u_copy.shape[1]):
            batch_u_step = batch_u_copy.clone()  # Clone to avoid modification issues
            batch_u_step[:, t, 4] = last_predictions  # Inject last predictions
            batch_u_tmp = batch_u_step[:, :t+1, :]  # Take relevant time slice

            # Forward pass
            last_predictions = model(batch_u_tmp)[:, -1, :].view(-1)  # Ensure shape matches

            batch_y_pred_list.append(last_predictions.unsqueeze(1))  # Store prediction

        # Concatenate all predictions along time dimension
        batch_y_pred = torch.cat(batch_y_pred_list, dim=1).unsqueeze(-1)  # Ensure shape matches batch_y

        # Compute loss
        loss = criterion(batch_y, batch_y_pred)

        # Backpropagation
        loss.backward()
        optimizer.step()

        running_loss += loss.item()

        # Debugging: Check if all parameters have gradients
        for name, param in model.named_parameters():
            if param.grad is None:
                print(f"Warning: No gradient computed for {name}")

    return running_loss / len(dataloader)



def validate(model, dataloader, criterion, device):
    model.eval()
    running_loss = 0.0
    with torch.no_grad():
        for batch in dataloader:
            batch_u, batch_y = batch
            batch_u, batch_y = batch_u.to(device), batch_y.to(device)

            batch_y_pred = torch.zeros_like(batch_y)
        
            # create a copy of batch_u to work with, then overwrite the real velocity (symbolic, may not be needed for the code)
            batch_u_copy = batch_u.clone().detach()
            batch_u_copy[:,:,4] = 0

            # simulate step by step
            last_predictions = torch.zeros(batch_u_copy.shape[0], device=device)

            for t in range(batch_u_copy.shape[1]):
                batch_u_step = batch_u_copy.clone()
                batch_u_step[:,t,4] = last_predictions
                batch_u_tmp = batch_u_step[:,:t+1,:]
                #update last predictions
                last_predictions = model(batch_u_tmp)[:,-1,:].view(-1)
                batch_y_pred[:,t,0] = last_predictions

            loss = criterion(batch_y, batch_y_pred)

            running_loss += loss.item()

    return running_loss / len(dataloader)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Meta system identification with transformers')

    # Overall
    parser.add_argument('--model-dir', type=str, default="out", metavar='S',
                        help='Saved model folder')
    parser.add_argument('--out-file', type=str, default=checkpoint_name_to_save, metavar='S',
                        help='Saved model name')
    parser.add_argument('--in-file', type=str, default=checkpoint_name_to_open, metavar='S',
                        help='Loaded model name (when resuming)')
    parser.add_argument('--init-from', type=str, default=mode, metavar='S',
                        help='Init from (scratch|resume|pretrained)')
    parser.add_argument('--seed', type=int, default=42, metavar='N',
                        help='Seed for random number generation')
    parser.add_argument('--log-wandb', action='store_true', default=False,
                        help='disables CUDA training')

    # Dataset
    parser.add_argument('--nx', type=int, default=4, metavar='N',
                        help='model order (default: 5)')
    parser.add_argument('--nu', type=int, default=5, metavar='N',
                        help='model order (default: 5)')
    parser.add_argument('--ny', type=int, default=1, metavar='N',
                        help='model order (default: 5)')
    parser.add_argument('--seq-len', type=int, default=sequence_length, metavar='N',
                        help='sequence length (default: 600)')
    parser.add_argument('--mag_range', type=tuple, default=(0.5, 0.97), metavar='N',
                        help='sequence length (default: 600)')
    parser.add_argument('--phase_range', type=tuple, default=(0.0, math.pi/2), metavar='N',
                        help='sequence length (default: 600)')
    parser.add_argument('--fixed-system', action='store_true', default=False,
                        help='If True, keep the same model all the times')

    # Model
    parser.add_argument('--n-layer', type=int, default=layers_number, metavar='N',
                        help='number of iterations (default: 1M)')
    parser.add_argument('--n-head', type=int, default=heads_number, metavar='N',
                        help='number of iterations (default: 1M)')
    parser.add_argument('--n-embd', type=int, default=embd_number, metavar='N',
                        help='number of iterations (default: 1M)')
    parser.add_argument('--dropout', type=float, default=0, metavar='LR',
                        help='learning rate (default: 1e-4)')
    parser.add_argument('--bias', action='store_true', default=False,
                        help='bias in model')

    # Training
    parser.add_argument('--batch-size', type=int, default=batch_size_, metavar='N',
                        help='batch size (default:32)')
    parser.add_argument('--max-iters', type=int, default= max_iteration_number, metavar='N',
                        help='number of iterations (default: 1M)')
    parser.add_argument('--warmup-iters', type=int, default=5_000, metavar='N',
                        help='number of iterations (default: 1000)')
    parser.add_argument('--lr', type=float, default=learning_rate_value, metavar='LR',
                        help='learning rate (default: 1e-4)')
    parser.add_argument('--weight-decay', type=float, default=0.0, metavar='D',
                        help='weight decay (default: 1e-4)')
    parser.add_argument('--eval-interval', type=int, default=10, metavar='N',
                        help='batch size (default:32)')
    parser.add_argument('--eval-iters', type=int, default=10, metavar='N',
                        help='batch size (default:32)')
    parser.add_argument('--fixed-lr', action='store_true', default=False,
                        help='disables CUDA training')

    # Compute
    parser.add_argument('--threads', type=int, default=16,
                        help='number of CPU threads (default: 10)')
    parser.add_argument('--no-cuda', action='store_true', default=False,
                        help='disables CUDA training')
    parser.add_argument('--cuda-device', type=str, default="cuda:0", metavar='S',
                        help='cuda device (default: "cuda")')
    parser.add_argument('--compile', action='store_true', default=False,
                        help='disables CUDA training')

    cfg = parser.parse_args()

    # Other settings
    cfg.beta1 = 0.9
    cfg.beta2 = 0.95

    print(cfg.seq_len)

    # Derived settings
    n_skip = 0
    cfg.block_size = cfg.seq_len
    cfg.lr_decay_iters = cfg.max_iters
    cfg.min_lr = cfg.lr/10.0  #
    cfg.decay_lr = not cfg.fixed_lr
    cfg.eval_batch_size = cfg.batch_size

    # Set seed for reproducibility
    torch.manual_seed(cfg.seed)
    np.random.seed(cfg.seed) # not needed? All randomness now handled with generators

    # Create out dir
    model_dir = Path(cfg.model_dir)
    model_dir.mkdir(exist_ok=True)

    # Configure compute
    cuda_device = "cuda:0"

    torch.set_num_threads(cfg.threads)
    use_cuda = not cfg.no_cuda and torch.cuda.is_available()
    device_name = cuda_device if use_cuda else "cpu"
    device = torch.device(device_name)
    device_type = 'cuda' if 'cuda' in device_name else 'cpu' # for later use in torch.autocast
    torch.set_float32_matmul_precision("high")
    torch.cuda.set_device(device)
    print(torch.cuda.is_available())
    print(torch.cuda.current_device())

    # Load all your DataFrames (replace with your data loading code)
    # folder_path = '../data/CL_experiments/train/inertia13_ki-0.0061-kp-11.8427'
    dfs = []
    for path_iter in folder_path_training:
        new_dfs = load_dataframes_from_folder(path_iter)
        dfs= dfs + new_dfs
        print(f"Loaded {len(new_dfs)} DataFrames from {path_iter}.")

    train_ds = Dataset(dfs=dfs, seq_len=cfg.seq_len)
    train_dl = DataLoader(train_ds, batch_size=cfg.batch_size)

    # if we work with a constant model we also validate with the same (thus same seed!)
    # val_ds = Dataset(dfs=dfs, seq_len=cfg.seq_len)
    # val_dl = DataLoader(val_ds, batch_size=cfg.eval_batch_size)

    ##########################

    #######here change the validation ds, put the real data
    ##### may god forgive us

    dfs_val = []
    for path_iter in folder_path_val:
        dfs_val = dfs_val + load_dataframes_from_folder(path_iter)
        print(f"Loaded {len(dfs_val)} DataFrames from {path_iter}.")

    val_ds = Dataset(dfs=dfs_val, seq_len=cfg.seq_len)
    # val_dl = DataLoader(val_ds, batch_size=cfg.eval_batch_size, pin_memory=True, num_workers=4, shuffle=True)
    val_dl = DataLoader(val_ds, batch_size=cfg.eval_batch_size, pin_memory=True, shuffle=True)
    #train_loader = DataLoader(train_dataset, batch_size=args.batch_size, pin_memory=True, num_workers=4, shuffle=True)


    print("saving model in: ", checkpoint_name_to_save)
    if mode != "scratch":
        print("starting from model: ", checkpoint_name_to_open, " (", mode, ")")
    print("sequence length: ", sequence_length)
    print("max iterations: ", max_iteration_number)
    print("batch size: ", batch_size_)
    print("learning rate: ", learning_rate_value)
    print("layers: ", layers_number)
    print("heads: ", heads_number)
    print("embd: ", embd_number)
    
    if weird_stuff:
        print("using experimental batch extractor")
    
    input("everything ok?")

    # Model
    model_args = dict(n_layer=cfg.n_layer, n_head=cfg.n_head, n_embd=cfg.n_embd, n_x=cfg.nx, n_y=cfg.ny, n_u=cfg.nu, block_size=cfg.block_size,
                      bias=cfg.bias, dropout=cfg.dropout)  # start with model_args from command line

    if cfg.init_from == "scratch":
        gptconf = GPTConfig(**model_args)
        model = GPT(gptconf)
    elif cfg.init_from == "resume" or cfg.init_from == "pretrained":
        ckpt_path = model_dir / f"{cfg.in_file}.pt"
        checkpoint = torch.load(ckpt_path, map_location=device, weights_only=False)
        gptconf = GPTConfig(**checkpoint["model_args"])
        model = GPT(gptconf)
        state_dict = checkpoint['model']
        # fix the keys of the state dictionary :(
        # honestly no idea how checkpoints sometimes get this prefix, have to debug more
        unwanted_prefix = '_orig_mod.'
        for k, v in list(state_dict.items()):
            if k.startswith(unwanted_prefix):
                state_dict[k[len(unwanted_prefix):]] = state_dict.pop(k)
        model.load_state_dict(state_dict)

    # Wrap the model with DataParallel
    if torch.cuda.device_count() > 1:
        print("Using all the GPUs!")
        model = nn.DataParallel(model)

    model.to(device)

    if cfg.compile:
        model = torch.compile(model)  # requires PyTorch 2.0

    # Optimizer
    # Check if model is wrapped by DataParallel
    if isinstance(model, torch.nn.DataParallel):
        optimizer = model.module.configure_optimizers(cfg.weight_decay, cfg.lr, (cfg.beta1, cfg.beta2), device_type)
    else:
        optimizer = model.configure_optimizers(cfg.weight_decay, cfg.lr, (cfg.beta1, cfg.beta2), device_type)

    if cfg.init_from == "resume":
        optimizer.load_state_dict(checkpoint['optimizer'])

    # Criterion
    criterion = torch.nn.MSELoss()

    # Training and validation loop
    LOSS_ITR = []
    LOSS_VAL = []
    best_val_loss = float('inf')

    if cfg.init_from == ("scratch") or cfg.init_from == "pretrained":
        # Training and validation loop
        LOSS_ITR = []
        LOSS_VAL = []
        iter_num = 0
        best_val_loss = np.inf
        train_time = 0.0
    elif cfg.init_from == "resume":
        # Training and validation loop
        LOSS_ITR = checkpoint['LOSS']
        LOSS_VAL = checkpoint['LOSS_VAL']
        iter_num = checkpoint["iter_num"]
        best_val_loss = checkpoint['best_val_loss']
        train_time = checkpoint['train_time']

    get_lr = partial(warmup_cosine_lr, lr=cfg.lr, min_lr=cfg.min_lr,
                     warmup_iters=cfg.warmup_iters, lr_decay_iters=cfg.lr_decay_iters)
    time_start = time.time()

    best_epoch = iter_num -1
    for epoch in range(iter_num+1, cfg.max_iters):
        ## I COMMENTED THIS PART BECAUSE THERE WAS A PROBLEM WITH LR : IT WAS STUCK TO 0
        if cfg.decay_lr:
            lr_iter = get_lr(epoch)
        else:
            lr_iter = cfg.lr
        optimizer.param_groups[0]['lr'] = lr_iter

        #scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=cfg.lr_decay_iters)
        train_loss = train(model, train_dl, criterion, optimizer, device)
        val_loss = validate(model, val_dl, criterion, device)
        #scheduler.step()
        # print("...")

        LOSS_ITR.append(train_loss)
        LOSS_VAL.append(val_loss)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_epoch = epoch
            checkpoint = {
                'model': model.module.state_dict() if isinstance(model, torch.nn.DataParallel) else model.state_dict(),
                'optimizer': optimizer.state_dict(),
                'model_args': model_args,
                'iter_num': epoch,
                'train_time': time.time() - time_start + train_time,
                'LOSS': LOSS_ITR,
                'LOSS_VAL': LOSS_VAL,
                'best_val_loss': best_val_loss,
                'cfg': cfg,
            }

            torch.save(checkpoint, model_dir / f"{cfg.out_file}.pt")

        
        print("model: ", checkpoint_name_to_save)
        print(f"Epoch [{epoch}], Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}, LR: {optimizer.param_groups[0]['lr']:.6f}, best val loss was: {best_val_loss:.4f}")
        wandb.log({"epoch": epoch, "loss": train_loss, "val_loss": val_loss, "best_epoch": best_epoch})

    print("Training complete. Best model saved as: ", checkpoint_name_to_save)
    checkpoint = {
                'model': model.module.state_dict() if isinstance(model, torch.nn.DataParallel) else model.state_dict(),
                'optimizer': optimizer.state_dict(),
                'model_args': model_args,
                'iter_num': cfg.max_iters,
                'train_time': time.time() - time_start + train_time,
                'LOSS': LOSS_ITR,
                'LOSS_VAL': LOSS_VAL,
                'best_val_loss': best_val_loss,
                'cfg': cfg,
    }

    torch.save(checkpoint, model_dir / f"{cfg.out_file}_loss_check.pt")