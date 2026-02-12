import asyncio
import logging
from typing import List, Any, Callable
from .schemas import DocumentExtraction

logger = logging.getLogger("arkhe.processor")

class ParallelDocumentProcessor:
    """
    Optimizes long document processing with parallel chunking (BLOCO 370).
    """
    def __init__(self, concurrency_limit: int = 5):
        self.semaphore = asyncio.Semaphore(concurrency_limit)

    async def process_chunks(self, chunks: List[Any], process_func: Callable) -> List[Any]:
        """
        Processes document chunks in parallel with concurrency management and error boundaries.
        """
        async def bounded_process(chunk, index):
            async with self.semaphore:
                try:
                    logger.info(f"Processing chunk {index+1}/{len(chunks)}...")
                    return await process_func(chunk)
                except Exception as e:
                    logger.error(f"Error processing chunk {index+1}: {e}")
                    return {"chunk_index": index, "error": str(e), "status": "FAILED"}

        tasks = [bounded_process(chunk, i) for i, chunk in enumerate(chunks)]
        results = await asyncio.gather(*tasks, return_exceptions=False)
        return results

    async def aggregate_results(self, results: List[DocumentExtraction]) -> DocumentExtraction:
        """
        Aggregates multiple extraction results into a single document state.
        """
        # Placeholder for complex aggregation logic
        if not results:
            return None

        # Filtering out failed chunks
        successful_results = [r for r in results if hasattr(r, 'entities')]
        if not successful_results:
            return None

        return successful_results[0]
