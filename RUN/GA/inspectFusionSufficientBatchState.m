function state = inspectFusionSufficientBatchState(plan)
% INSPECTFUSIONSUFFICIENTBATCHSTATE Classify immutable batch ledger state.

reservationExists = exist(plan.attemptDirectory, 'dir') == 7;
workerDirectoryExists = exist(plan.workerDirectory, 'dir') == 7;
attemptSuccess = exist(plan.attemptSuccessReceiptPath, 'file') == 2;
workerSuccess = exist(plan.workerSuccessReceiptPath, 'file') == 2;
attemptFailed = exist(plan.attemptFailedTombstonePath, 'file') == 2;
workerFailed = exist(plan.workerFailedTombstonePath, 'file') == 2;
reservationReceipt = exist(plan.reservationReceiptPath, 'file') == 2;

if ~reservationExists
    if workerDirectoryExists || attemptSuccess || workerSuccess || ...
            attemptFailed || workerFailed
        name = 'ORPHANED_BURNED';
    else
        name = 'UNRESERVED';
    end
elseif attemptFailed || workerFailed
    name = 'FAILED_BURNED';
elseif attemptSuccess && workerSuccess && reservationReceipt && ...
        workerDirectoryExists
    name = 'COMPLETE_WORKERS';
elseif attemptSuccess || workerSuccess
    name = 'INCONSISTENT_BURNED';
else
    name = 'RESERVED_BURNED';
end
state = struct( ...
    'name', name, ...
    'reservationExists', reservationExists, ...
    'reservationReceiptExists', reservationReceipt, ...
    'workerDirectoryExists', workerDirectoryExists, ...
    'attemptSuccessExists', attemptSuccess, ...
    'workerSuccessExists', workerSuccess, ...
    'attemptFailedExists', attemptFailed, ...
    'workerFailedExists', workerFailed);
end
