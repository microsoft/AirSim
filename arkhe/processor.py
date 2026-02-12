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

    def chunk_text(self, text: str, chunk_size: int = 2000, overlap: int = 200) -> List[str]:
        """
        Splits long text into overlapping chunks for processing (BLOCO 370).
        """
        chunks = []
        start = 0
        while start < len(text):
            end = start + chunk_size
            chunks.append(text[start:end])
            start += (chunk_size - overlap)
        return chunks

    async def aggregate_results(self, results: List[DocumentExtraction]) -> DocumentExtraction:
        """
        Aggregates multiple extraction results into a single document state.
        """
        if not results:
            return None

        successful_results = [r for r in results if hasattr(r, 'entities') and r.entities]
        if not successful_results:
            return results[0] if results else None

        # Merging entities from all successful chunks
        merged_entities = []
        for r in successful_results:
            merged_entities.extend(r.entities)

        # Creating a unified result based on the first successful one
        base = successful_results[0]
        return DocumentExtraction(
            document_hash=base.document_hash,
            entities=merged_entities,
            layout=base.layout,
            psi=base.psi,
            satoshi_cost=sum(r.satoshi_cost for r in successful_results),
            status="AGGREGATED"
        )
